"""Numpy deployment core of the Isaac Lab drop policy -- no ROS, no torch.

Mirrors, line for line, what the Isaac task env (isaac_lab/drone_bombard/task_env.py) computes:
  * observation (26 ch)            -> TaskEnv._get_observations (no sensor noise at deployment)
  * action processing              -> TaskEnv._pre_physics_step (clip, rate limit 0.2, scale 4/3/3/1)
  * nominal impact predictor       -> DroneBombardEnv._nominal_impact (latency-propagated, drag 0.07)
  * learned residual (GRU + EMA)   -> play.py _SLResidual, impact = nominal + clip(d, +-2 m)
  * release decision               -> TaskEnv._resolve_release (gate & wants_drop & first crossing)
World frame is ENU (x East, y North, z Up), body frame FLU -- the Isaac convention. ``frames``
converts from PX4 NED/FRD. Golden tests: test/test_policy_core.py (vs the torch originals).
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

# ---------------------------------------------------------------- constants (Isaac cfg values)
G = 9.81
ACT_SCALE = np.array([4.0, 3.0, 3.0, 1.0])     # ActionCfg vx/vy/vz/yaw scales
RATE_LIMIT = 0.2                                # ActionCfg.rate_limit
DROP_THRESHOLD = 0.5                            # action[4] > 0.5 -> wants drop
RELEASE_DELAY_MEAN = 0.22                       # ModelErrCfg.release_delay_mean (predictor models the mean)
PAYLOAD_MOUNT_Z = -0.14                         # payload hangs 0.14 m below the body origin
PAYLOAD_GROUND_Z, PAYLOAD_LAND_EPS = 0.0, 0.10  # landing latch plane
DRAG_COEF_NOMINAL = 0.07                        # DropCfg.payload_drag_coef_nominal
RESIDUAL_SCALE = 2.0                            # ResidualCfg.oracle_scale (play.py SL injection)
REVEAL_RADIUS = 7.0                             # PerceptionCfg.reveal_radius
EPISODE_STEPS = 300                             # 30 s at 10 Hz
GATE = dict(radius=1.5, alt_min=3.0, alt_max=8.0, max_speed=5.0, max_vz=3.0, max_tilt=0.35, max_ang_vel=4.0)


# ---------------------------------------------------------------- frames
_NED2ENU = np.array([[0.0, 1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, -1.0]])
_FLU2FRD = np.diag([1.0, -1.0, -1.0])


def quat_to_rot(q_wxyz) -> np.ndarray:
    w, x, y, z = q_wxyz
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)]])


def euler_xyz(R: np.ndarray) -> tuple[float, float, float]:
    """Isaac Lab euler_xyz_from_quat on a rotation matrix (roll, pitch, yaw in (-pi, pi])."""
    roll = math.atan2(R[2, 1], R[2, 2])
    pitch = math.asin(max(-1.0, min(1.0, -R[2, 0])))
    yaw = math.atan2(R[1, 0], R[0, 0])
    return roll, pitch, yaw


def px4_to_enu(pos_ned, vel_ned, q_ned_frd_wxyz, angvel_frd):
    """PX4 vehicle_odometry (NED world, FRD body) -> ENU world / FLU body, as the Isaac env sees it."""
    pos = _NED2ENU @ np.asarray(pos_ned, float)
    vel = _NED2ENU @ np.asarray(vel_ned, float)
    R = _NED2ENU @ quat_to_rot(q_ned_frd_wxyz) @ _FLU2FRD
    ang = _FLU2FRD @ np.asarray(angvel_frd, float)
    return pos, vel, R, ang


def enu_vel_to_ned(v_enu_xyz, yaw_rate_enu):
    """ENU velocity + CCW yaw rate -> PX4 NED velocity + yaw rate (CW positive)."""
    v = _NED2ENU @ np.asarray(v_enu_xyz, float)   # the swap matrix is its own inverse
    return v, -float(yaw_rate_enu)


# ---------------------------------------------------------------- networks
def _elu(x):
    return np.where(x > 0, x, np.expm1(np.minimum(x, 0.0)))


def _sigmoid(x):
    return 1.0 / (1.0 + np.exp(-x))


class Actor:
    """rsl_rl actor 26 -> 256 -> 256 -> 7, ELU. Deterministic (mean) action."""

    def __init__(self, m):
        self.W = [m["actor_0_weight"], m["actor_2_weight"], m["actor_4_weight"]]
        self.b = [m["actor_0_bias"], m["actor_2_bias"], m["actor_4_bias"]]

    def __call__(self, obs):
        h = _elu(self.W[0] @ obs + self.b[0])
        h = _elu(self.W[1] @ h + self.b[1])
        return self.W[2] @ h + self.b[2]


class GRUResidual:
    """_fit_sl_seq Exported: standardize -> GRUCell(26->64) -> Linear(64->2). Output in metres."""

    def __init__(self, m):
        self.w_ih, self.w_hh = m["gru_w_ih"], m["gru_w_hh"]
        self.b_ih, self.b_hh = m["gru_b_ih"], m["gru_b_hh"]
        self.hw, self.hb = m["gru_head_w"], m["gru_head_b"]
        self.mu, self.sd = m["gru_mu"], m["gru_sd"]
        self.h = np.zeros(self.w_hh.shape[1])

    def reset(self):
        self.h[:] = 0.0

    def __call__(self, obs):
        x = (obs - self.mu) / self.sd
        H = self.h.shape[0]
        gi, gh = self.w_ih @ x + self.b_ih, self.w_hh @ self.h + self.b_hh
        r = _sigmoid(gi[:H] + gh[:H])
        z = _sigmoid(gi[H:2 * H] + gh[H:2 * H])
        n = np.tanh(gi[2 * H:] + r * gh[2 * H:])
        self.h = (1.0 - z) * n + z * self.h
        return self.hw @ self.h + self.hb


# ---------------------------------------------------------------- ballistics
def time_to_fall(alt, vz, g=G):
    h = max(alt, 0.0)
    return max((vz + math.sqrt(vz * vz + 2.0 * g * h)) / g, 0.0)


def nominal_impact(pos_xy, vel_xy, vz, alt):
    """DroneBombardEnv._nominal_impact: propagate over the mean release latency, fall the payload's
    geometry, first-order drag on the inherited velocity. No wind (that is the residual's job)."""
    tau = RELEASE_DELAY_MEAN
    p = np.asarray(pos_xy, float) + np.asarray(vel_xy, float) * tau
    a = alt + vz * tau
    fall = a + PAYLOAD_MOUNT_Z - (PAYLOAD_GROUND_Z + PAYLOAD_LAND_EPS)
    tf = time_to_fall(fall, vz)
    return p + np.asarray(vel_xy, float) * tf - DRAG_COEF_NOMINAL * np.asarray(vel_xy, float) * tf * tf


def release_gate(d_impact, alt, speed_xy, vz, roll, pitch, ang_norm, attached):
    c = GATE
    return bool(d_impact <= c["radius"] and c["alt_min"] <= alt <= c["alt_max"] and speed_xy <= c["max_speed"]
                and abs(vz) <= c["max_vz"] and abs(roll) <= c["max_tilt"] and abs(pitch) <= c["max_tilt"]
                and ang_norm <= c["max_ang_vel"] and attached)


# ---------------------------------------------------------------- policy state machine piece
@dataclass
class DropPolicy:
    """One episode of the Isaac policy loop. Call ``reset`` at handoff, ``step`` at 10 Hz,
    ``release_check`` at the state-estimate rate (Isaac resolves the release at 100 Hz)."""
    models: dict
    target_xy: np.ndarray
    use_residual: bool = True
    ema_alpha: float = 0.3
    prev_action: np.ndarray = field(default_factory=lambda: np.zeros(4))
    residual_m: np.ndarray = field(default_factory=lambda: np.zeros(2))

    def __post_init__(self):
        self.actor = Actor(self.models)
        self.gru = GRUResidual(self.models)
        self.reset(np.zeros(3))

    def reset(self, cruise_vel_enu):
        """Handoff. Isaac seeds prev_action (and the LPF) at the cruise velocity command."""
        self.prev_action = np.clip(np.array([cruise_vel_enu[0] / ACT_SCALE[0], cruise_vel_enu[1] / ACT_SCALE[1], 0.0, 0.0]), -1, 1)
        self.gru.reset()
        self.residual_m = np.zeros(2)
        self._fresh = True
        self.step_count = 0
        self.wants_drop = False
        self.released = False
        self.attached = True
        self._d_prev = math.inf

    # -- aiming solution: (corrected impact - target), |.|, t_fall
    def ccip(self, pos, vel):
        nom = nominal_impact(pos[:2], vel[:2], vel[2], pos[2])
        imp = nom + (np.clip(self.residual_m, -RESIDUAL_SCALE, RESIDUAL_SCALE) if self.use_residual else 0.0)
        err = imp - self.target_xy
        return err, float(np.linalg.norm(err)), time_to_fall(pos[2], vel[2])

    def observation(self, pos, vel, R, ang):
        roll, pitch, yaw = euler_xyz(R)
        alt = pos[2]
        err, d_imp, tf = self.ccip(pos, vel)
        d_xy = float(np.linalg.norm(self.target_xy - pos[:2]))
        det = d_xy <= REVEAL_RADIUS
        rel = self.target_xy - pos[:2]
        c = lambda x, s: float(np.clip(x / s, -1.0, 1.0))  # noqa: E731
        m = 1.0 if det else 0.0
        marker = [c(rel[0], 20.0) * m, c(rel[1], 20.0) * m, c(err[0], 10.0) * m, c(err[1], 10.0) * m,
                  float(np.clip(d_imp / 10.0, 0.0, 1.0)) * m]
        own = [c(0.0 - alt, 20.0), c(vel[0], 10.0), c(vel[1], 10.0), c(vel[2], 10.0),
               float(np.clip(roll / math.pi, -1, 1)), float(np.clip(pitch / math.pi, -1, 1)),
               math.sin(yaw), math.cos(yaw),
               c(ang[0], math.pi), c(ang[1], math.pi), c(ang[2], math.pi),
               float(np.clip(tf / 5.0, 0.0, 1.0)), float(np.clip(np.linalg.norm(vel[:2]) / 10.0, 0.0, 1.0)),
               float(np.clip(alt / 20.0, 0.0, 1.0)), 1.0 if self.attached else 0.0,
               float(np.clip(1.0 - self.step_count / EPISODE_STEPS, 0.0, 1.0))]
        return np.array(marker + own + list(self.prev_action) + [1.0 if det else 0.0], dtype=np.float32)

    def step(self, pos, vel, R, ang):
        """10 Hz policy step -> (velocity cmd ENU [vx,vy,vz], yaw rate). Updates residual and drop intent."""
        obs = self.observation(pos, vel, R, ang)
        a = np.clip(self.actor(obs), -1.0, 1.0)
        lim = np.clip(a[:4], self.prev_action - RATE_LIMIT, self.prev_action + RATE_LIMIT)
        self.wants_drop = bool(a[4] > DROP_THRESHOLD)
        if self.use_residual:
            d = self.gru(obs)
            self.residual_m = d if self._fresh else self.ema_alpha * d + (1.0 - self.ema_alpha) * self.residual_m
        self._fresh = False
        self.prev_action = lim
        self.step_count += 1
        cmd = lim * ACT_SCALE
        return cmd[:3], float(cmd[3]), obs

    def release_check(self, pos, vel, R, ang) -> bool:
        """TaskEnv._resolve_release at the estimate rate. True exactly once: when to command the drop."""
        if self.released:
            return False
        roll, pitch, _ = euler_xyz(R)
        _, d_imp, _ = self.ccip(pos, vel)
        gate = release_gate(d_imp, pos[2], float(np.linalg.norm(vel[:2])), vel[2], roll, pitch,
                            float(np.linalg.norm(ang)), self.attached)
        crossed = d_imp >= self._d_prev
        self._d_prev = d_imp
        if self.wants_drop and gate and crossed:
            self.released = True
            return True
        return False
