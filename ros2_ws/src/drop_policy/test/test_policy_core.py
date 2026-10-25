"""Numpy port vs the torch originals (golden pairs written by isaac_lab/_export_deploy.py)."""
import os
import math
import numpy as np
from drop_policy import policy_core as pc

M = np.load(os.path.join(os.path.dirname(__file__), "..", "models", "drop_policy_models.npz"))


def test_actor_matches_torch():
    a = pc.Actor(M)
    out = np.stack([a(o) for o in M["golden_obs"]])
    assert np.abs(out - M["golden_act"]).max() < 1e-4


def test_gru_sequence_matches_torch():
    g = pc.GRUResidual(M)
    out = np.stack([g(o) for o in M["golden_obs"]])
    assert np.abs(out - M["golden_gru"]).max() < 1e-4


def test_nominal_impact_matches_isaac():
    for i in range(len(M["golden_pos"])):
        nom = pc.nominal_impact(M["golden_pos"][i], M["golden_vel"][i], M["golden_vz"][i], M["golden_alt"][i])
        assert np.abs(nom - M["golden_nom"][i]).max() < 1e-3
        assert abs(pc.time_to_fall(M["golden_alt"][i], M["golden_vz"][i]) - M["golden_tf"][i]) < 1e-4


def test_frames_level_hover_facing_east():
    # PX4: yaw 90 deg (facing East) level; NED velocity 1 m/s East -> ENU vx = 1, yaw_enu = 0
    q = (math.cos(math.pi / 4), 0.0, 0.0, math.sin(math.pi / 4))
    pos, vel, R, ang = pc.px4_to_enu([2.0, 3.0, -10.0], [0.0, 1.0, 0.5], q, [0.1, 0.2, 0.3])
    assert np.allclose(pos, [3.0, 2.0, 10.0]) and np.allclose(vel, [1.0, 0.0, -0.5])
    r, p, y = pc.euler_xyz(R)
    assert abs(r) < 1e-9 and abs(p) < 1e-9 and abs(y) < 1e-9
    assert np.allclose(ang, [0.1, -0.2, -0.3])


def test_pitch_sign_nose_down_forward():
    # FRD pitch -10 deg (nose down) facing North -> FLU pitch +10 deg in Isaac's xyz convention
    th = math.radians(-10)
    q = (math.cos(th / 2), 0.0, math.sin(th / 2), 0.0)
    _, _, R, _ = pc.px4_to_enu([0, 0, 0], [0, 0, 0], q, [0, 0, 0])
    r, p, y = pc.euler_xyz(R)
    assert abs(math.degrees(p) - 10.0) < 1e-6 and abs(math.degrees(y) - 90.0) < 1e-6
