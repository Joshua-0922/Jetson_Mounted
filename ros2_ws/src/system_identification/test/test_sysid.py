import math

import pytest

from system_identification import maneuvers, safety
from system_identification.frames import TestFrame as Frame


@pytest.mark.parametrize('test', sorted(maneuvers.TESTS))
@pytest.mark.parametrize('hover_z', [1.2, 1.5, 1.7])
def test_every_plan_stays_in_envelope(test, hover_z):
    segs = maneuvers.build(test, hover_z)
    assert segs[0].name == 'takeoff' and segs[-1].name == 'final_hover'
    for _, seg, sp, (x, y, z) in maneuvers.sample(segs):
        assert z <= maneuvers.PLAN_MAX_Z < safety.Limits().max_alt <= safety.HARD_MAX_ALT
        assert max(abs(x), abs(y)) <= maneuvers.PLAN_MAX_XY < safety.Limits().box_xy


def test_plans_are_continuous():
    """Consecutive position setpoints never jump more than the designed steps (1.5 m)."""
    segs = maneuvers.build('full', 1.5)
    prev = None
    for _, seg, sp, xyz in maneuvers.sample(segs, rate=10):
        if prev is not None:
            assert math.dist(prev, xyz) <= 1.5 + 1e-6, seg.name
        prev = xyz


def test_chirp_acceleration_limited():
    p = maneuvers.Plan((0, 0, 1.5))
    p.chirp('x', 0.1, 1.0, 40.0, amp=1.0, a_max=3.0, name='c')
    seg = p.segments[0]
    dt = 0.01
    xs = [seg.at(k * dt).x for k in range(int(40 / dt) + 1)]
    acc = max(abs(xs[k + 1] - 2 * xs[k] + xs[k - 1]) / dt ** 2 for k in range(1, len(xs) - 1))
    assert acc < 3.0 * 1.3          # amplitude cut + taper slope
    assert abs(xs[0]) < 1e-9 and abs(xs[-1]) < 1e-6


def test_envelope_rejects_bad_hover():
    with pytest.raises(ValueError):
        maneuvers.build('vertical', 2.5)


def test_velocity_segments_hold_altitude():
    segs = maneuvers.build('velocity', 1.5)
    runs = [s for s in segs if s.name.endswith('_run')]
    assert len(runs) == 6
    for s in runs:
        sp = s.at(0.5 * s.duration)
        assert sp.xy_velocity_mode and sp.z == pytest.approx(1.5)
        assert abs(sp.vx) + abs(sp.vy) in (1.0, 2.0)


@pytest.mark.parametrize('heading', [0.0, 0.7, -2.5, math.pi])
def test_frame_roundtrip(heading):
    f = Frame(3.0, -4.0, -0.2, heading)
    for p in [(1.0, 0.0, 1.5), (0.0, 1.0, 0.0), (-2.0, 3.0, 2.0)]:
        assert f.pos_from_ned(*f.pos_to_ned(*p)) == pytest.approx(p)
        assert f.vel_from_ned(*f.vel_to_ned(*p)) == pytest.approx(p)
    # forward = start heading; left = 90 deg CCW (towards -East when facing North)
    n, e, d = f.pos_to_ned(1.0, 0.0, 1.0)
    assert (n - 3.0, e + 4.0) == pytest.approx((math.cos(heading), math.sin(heading)))
    assert d == pytest.approx(-1.2)
    if heading == 0.0:
        assert f.pos_to_ned(0.0, 1.0, 0.0)[:2] == pytest.approx((3.0, -5.0))
    assert f.yaw_from_ned(f.yaw_to_ned(0.3)) == pytest.approx(0.3)


def test_safety_check():
    lim = safety.Limits(max_alt=5.0)
    assert lim.max_alt == safety.HARD_MAX_ALT
    sp = maneuvers.Setpoint(0.0, 0.0, 1.5)
    ok = dict(roll=0.0, pitch=0.0, telem_age=0.1, sp=sp)
    assert safety.check(lim, (0, 0, 1.5), **ok) is None
    assert 'altitude' in safety.check(lim, (0, 0, 3.1), **ok)
    assert 'box' in safety.check(lim, (6.5, 0, 1.5), **ok)
    assert 'tilt' in safety.check(lim, (0, 0, 1.5), **{**ok, 'roll': math.radians(40)})
    assert 'telemetry' in safety.check(lim, (0, 0, 1.5), **{**ok, 'telem_age': 2.0})
    assert 'tracking' in safety.check(lim, (0, 2.8, 1.5), **ok)
    vsp = maneuvers.Setpoint(None, None, 1.5, vx=2.0)
    assert safety.check(lim, (0, 2.8, 1.5), 0.0, 0.0, 0.1, vsp) is None


def test_param_review():
    good = {'COM_OBL_RC_ACT': 4, 'COM_OF_LOSS_T': 1.0, 'GF_ACTION': 5, 'GF_MAX_VER_DIST': 3.0,
            'GF_MAX_HOR_DIST': 15.0, 'NAV_RCL_ACT': 3, 'COM_LOW_BAT_ACT': 2,
            'SDLOG_PROFILE': 25, 'NAV_DLL_ACT': 0, 'RTL_RETURN_ALT': 3.0, 'MC_BAT_SCALE_EN': 0}
    assert safety.review_params(good) == ([], [])
    px4_defaults = {**good, 'COM_OBL_RC_ACT': 0, 'GF_ACTION': 2, 'NAV_RCL_ACT': 2,
                    'GF_MAX_VER_DIST': 0.0, 'COM_LOW_BAT_ACT': 0}
    errors, _ = safety.review_params(px4_defaults)
    assert len(errors) == 5
