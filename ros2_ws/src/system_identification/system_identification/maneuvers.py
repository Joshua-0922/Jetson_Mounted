"""System-identification maneuvers as time-indexed setpoint segments (test frame, see frames.py).

Each test = takeoff -> body -> return to hover point. The node lands with PX4 AUTO.LAND afterwards.
Setpoints are evaluated at 10 Hz. Position segments also carry a velocity feed-forward, so PX4
tracks the sinusoids without the phase lag of a pure position loop.

Print a plan without flying:  ros2 run system_identification sysid_plan --test full
"""
import argparse
import math
from dataclasses import dataclass
from typing import Callable, Optional, Tuple

# Planning envelope. The node's runtime abort limits (safety.py) sit outside this envelope,
# and the PX4 geofence (GF_MAX_VER_DIST <= 3.0 m) sits outside those.
PLAN_MAX_Z = 2.5     # m above ground
PLAN_MIN_Z = 0.5     # m above ground, except takeoff
PLAN_MAX_XY = 4.5    # m from the takeoff point, per axis (including the predicted braking distance)
HOVER_Z_RANGE = (1.2, 1.7)

AXES = {'x': 0, 'y': 1, 'z': 2}


@dataclass(frozen=True)
class Setpoint:
    x: Optional[float]   # None (with y None) -> horizontal velocity control, z still position-held
    y: Optional[float]
    z: float
    vx: float = 0.0
    vy: float = 0.0
    vz: float = 0.0
    yaw: float = 0.0     # rad, relative to the start heading, CCW positive

    @property
    def xy_velocity_mode(self):
        return self.x is None


@dataclass
class Segment:
    name: str
    duration: float
    fn: Callable[[float, Tuple[float, float, float]], Setpoint]
    start: Tuple[float, float, float]          # planned start position
    latch_measured: bool = False               # start from the measured position instead
    ground: bool = False                       # takeoff: may be below PLAN_MIN_Z
    predicted_end: Optional[Tuple[float, float, float]] = None  # for velocity segments

    def at(self, t, p0=None):
        return self.fn(min(max(t, 0.0), self.duration), self.start if p0 is None else p0)


def _min_jerk(tau):
    tau = min(max(tau, 0.0), 1.0)
    s = tau ** 3 * (10 - 15 * tau + 6 * tau ** 2)
    ds = 30 * tau ** 2 * (1 - tau) ** 2
    return s, ds


def _replace(p, axis, value):
    q = list(p)
    q[axis] = value
    return tuple(q)


class Plan:
    """Appends segments and keeps track of the planned position (cursor) and yaw."""

    def __init__(self, start=(0.0, 0.0, 0.0)):
        self.segments = []
        self.p = tuple(float(v) for v in start)
        self.yaw = 0.0
        self._latch_next = False

    def _add(self, name, duration, fn, end, **kw):
        seg = Segment(name, float(duration), fn, self.p, latch_measured=self._latch_next, **kw)
        self._latch_next = False
        self.segments.append(seg)
        self.p = tuple(end)
        return seg

    def hold(self, duration, name='hold'):
        yaw = self.yaw
        return self._add(name, duration, lambda t, p0: Setpoint(*p0, yaw=yaw), self.p)

    def goto(self, target, name='goto', speed=0.5, yaw=None, min_duration=2.0, ground=False,
             latch=False):
        """Minimum-jerk move; `speed` is the peak speed."""
        target = tuple(float(v) for v in target)
        yaw0, yaw1 = self.yaw, (self.yaw if yaw is None else yaw)
        dist = math.dist(self.p, target)
        dur = max(min_duration, 1.875 * dist / speed, 1.875 * abs(yaw1 - yaw0) / math.radians(30))
        if latch:
            self._latch_next = True

        def fn(t, p0):
            s, ds = _min_jerk(t / dur)
            pos = [a + (b - a) * s for a, b in zip(p0, target)]
            vel = [(b - a) * ds / dur for a, b in zip(p0, target)]
            return Setpoint(*pos, *vel, yaw=yaw0 + (yaw1 - yaw0) * s)

        seg = self._add(name, dur, fn, target, ground=ground)
        self.yaw = yaw1
        return seg

    def steps(self, axis, offsets, dwell, name):
        """Position steps around the current point; each offset is held for `dwell` seconds."""
        i = AXES[axis]
        yaw = self.yaw
        base = self.p
        for k, off in enumerate(offsets):
            target = _replace(base, i, base[i] + off)
            self._add(f'{name}_{k}', dwell,
                      lambda t, p0, i=i, off=off: Setpoint(*_replace(p0, i, p0[i] + off), yaw=yaw),
                      target)
            # every step is defined relative to the same base, so pin the start of the next one
            self.p = base
        self.p = _replace(base, i, base[i] + offsets[-1])

    def chirp(self, axis, f0, f1, duration, amp, a_max, name, taper=2.0):
        """Linear chirp f0 -> f1 Hz. Amplitude is cut to a_max/(2 pi f)^2 so the commanded
        acceleration never exceeds a_max (m/s^2); cosine taper at both ends."""
        i = AXES[axis]
        yaw = self.yaw

        def fn(t, p0):
            f = f0 + (f1 - f0) * t / duration
            phase = 2 * math.pi * (f0 * t + 0.5 * (f1 - f0) * t * t / duration)
            w = 2 * math.pi * f
            a = min(amp, a_max / (w * w))
            edge = min(t, duration - t)
            if edge < taper:
                a *= 0.5 * (1 - math.cos(math.pi * max(edge, 0.0) / taper))
            pos = _replace(p0, i, p0[i] + a * math.sin(phase))
            vel = _replace((0.0, 0.0, 0.0), i, a * w * math.cos(phase))
            return Setpoint(*pos, *vel, yaw=yaw)

        self._add(name, duration, fn, self.p)

    def yaw_steps(self, angles_deg, dwell, name='yaw'):
        for k, ang in enumerate(angles_deg):
            yaw = math.radians(ang)
            self._add(f'{name}_{k}', dwell, lambda t, p0, yaw=yaw: Setpoint(*p0, yaw=yaw), self.p)
            self.yaw = yaw

    def velocity_run(self, axis, v, dist, name, brake=2.5):
        """Horizontal velocity step (what the drop policy commands): position at the run start,
        then velocity `v` for dist/|v| s, then velocity 0 (step down). z stays position-held."""
        i = AXES[axis]
        assert i in (0, 1)
        sign = 1.0 if v > 0 else -1.0
        start = _replace(self.p, i, -sign * dist / 2)
        self.goto(start, name=f'{name}_approach', speed=1.0)
        self.hold(2.0, name=f'{name}_settle')
        run_t = dist / abs(v)
        z = self.p[2]
        yaw = self.yaw

        def run(t, p0, v=v):
            vel = _replace((0.0, 0.0, 0.0), i, v)
            return Setpoint(None, None, z, *vel, yaw=yaw)

        end = _replace(start, i, start[i] + v * run_t)
        seg = self._add(f'{name}_run', run_t, run, end)
        # braking distance margin: ~0.5 s of the run speed (PX4 velocity P 1.8 -> ~3.6 m/s^2)
        seg.predicted_end = _replace(end, i, end[i] + 0.5 * v)
        seg = self._add(f'{name}_brake', brake,
                        lambda t, p0: Setpoint(None, None, z, yaw=yaw), end)
        seg.predicted_end = _replace(end, i, end[i] + 0.5 * v)
        self._latch_next = True  # continue from wherever the vehicle actually stopped


# ---------------------------------------------------------------------------------------------
# Tests

def _vertical(p):
    p.steps('z', [+0.8, 0.0, -0.6, 0.0], dwell=5.0, name='z_step')
    p.chirp('z', 0.1, 1.5, 40.0, amp=0.4, a_max=2.5, name='z_chirp')
    p.hold(5.0)


def _horizontal(p):
    for ax in ('x', 'y'):
        p.steps(ax, [+1.5, 0.0, -1.5, 0.0], dwell=6.0, name=f'{ax}_step')
    for ax in ('x', 'y'):
        p.chirp(ax, 0.1, 1.0, 40.0, amp=1.0, a_max=3.0, name=f'{ax}_chirp')
        p.hold(3.0)


def _yaw(p):
    p.yaw_steps([45, 0, -45, 0, 90, 0, -90, 0], dwell=5.0)


def _velocity(p):
    base = p.p
    for ax, v, d in (('x', 1.0, 4.0), ('x', -1.0, 4.0), ('x', 2.0, 6.0), ('x', -2.0, 6.0),
                     ('y', 1.0, 4.0), ('y', -1.0, 4.0)):
        p.velocity_run(ax, v, d, name=f'vel_{ax}{v:+.0f}')
    p.goto(base, name='vel_return', speed=1.0, latch=True)


def _hover(p):
    p.hold(30.0, name='hover')


def _battery(p):
    # 15 x 60 s blocks; the node lands earlier when the cell voltage reaches land_cell_v
    for k in range(15):
        p.hold(40.0, name=f'bat{k}_hover')
        p.steps('z', [+0.5, 0.0, -0.5, 0.0], dwell=5.0, name=f'bat{k}_z')


def _full(p):
    p.hold(15.0, name='hover_a')
    _vertical(p)
    _horizontal(p)
    _yaw(p)
    p.hold(5.0)
    _velocity(p)
    p.hold(15.0, name='hover_b')


TESTS = {
    'hover': _hover,
    'vertical': _vertical,
    'horizontal': _horizontal,
    'yaw': _yaw,
    'velocity': _velocity,
    'battery': _battery,
    'full': _full,
}


def build(test, hover_z=1.5, pilot_takeoff=False):
    if test not in TESTS:
        raise ValueError(f'unknown test {test!r}; choose from {sorted(TESTS)}')
    if not HOVER_Z_RANGE[0] <= hover_z <= HOVER_Z_RANGE[1]:
        raise ValueError(f'hover_z {hover_z} outside {HOVER_Z_RANGE}')
    p = Plan()
    # pilot_takeoff: vehicle is already hovering, start from wherever it is
    p.goto((0.0, 0.0, hover_z), name='takeoff', speed=0.5, ground=True, latch=pilot_takeoff)
    p.hold(5.0, name='settle')
    TESTS[test](p)
    p.goto((0.0, 0.0, hover_z), name='return', speed=0.8, yaw=0.0)
    p.hold(3.0, name='final_hover')
    validate(p.segments)
    return p.segments


def sample(segments, rate=10.0):
    """Yield (t_total, segment, setpoint, predicted_xyz) over the planned start positions."""
    t0 = 0.0
    for seg in segments:
        n = max(1, int(round(seg.duration * rate)))
        for k in range(n + 1):
            t = seg.duration * k / n
            sp = seg.at(t)
            if sp.xy_velocity_mode:
                frac = t / seg.duration
                a, b = seg.start, seg.predicted_end or seg.start
                xyz = tuple(u + (w - u) * frac for u, w in zip(a, b))
            else:
                xyz = (sp.x, sp.y, sp.z)
            yield t0 + t, seg, sp, xyz
        t0 += seg.duration


def validate(segments):
    for t, seg, sp, (x, y, z) in sample(segments):
        bad = None
        if z > PLAN_MAX_Z + 1e-9:
            bad = f'z={z:.2f} > {PLAN_MAX_Z}'
        elif z < PLAN_MIN_Z - 1e-9 and not seg.ground:
            bad = f'z={z:.2f} < {PLAN_MIN_Z}'
        elif max(abs(x), abs(y)) > PLAN_MAX_XY + 1e-9:
            bad = f'|xy|=({x:.2f},{y:.2f}) > {PLAN_MAX_XY}'
        elif seg.duration <= 0:
            bad = 'non-positive duration'
        if bad:
            raise ValueError(f'plan leaves the envelope in {seg.name} at t={t:.1f}s: {bad}')


def total_duration(segments):
    return sum(s.duration for s in segments)


def main():
    ap = argparse.ArgumentParser(description='print a system-identification plan (no flight)')
    ap.add_argument('--test', default='full', choices=sorted(TESTS))
    ap.add_argument('--hover-z', type=float, default=1.5)
    ap.add_argument('--csv', help='write the 10 Hz setpoints to this file')
    a = ap.parse_args()
    segs = build(a.test, a.hover_z)
    t = 0.0
    for s in segs:
        print(f'{t:7.1f}s  {s.duration:5.1f}s  {s.name}')
        t += s.duration
    print(f'total {t:.0f} s ({t / 60:.1f} min) + landing; envelope OK')
    if a.csv:
        with open(a.csv, 'w') as f:
            f.write('t,segment,x,y,z,vx,vy,vz,yaw,pred_x,pred_y,pred_z\n')
            for tt, seg, sp, xyz in sample(segs):
                xs = '' if sp.x is None else f'{sp.x:.4f}'
                ys = '' if sp.y is None else f'{sp.y:.4f}'
                f.write(f'{tt:.2f},{seg.name},{xs},{ys},{sp.z:.4f},{sp.vx:.4f},{sp.vy:.4f},'
                        f'{sp.vz:.4f},{sp.yaw:.4f},{xyz[0]:.4f},{xyz[1]:.4f},{xyz[2]:.4f}\n')


if __name__ == '__main__':
    main()
