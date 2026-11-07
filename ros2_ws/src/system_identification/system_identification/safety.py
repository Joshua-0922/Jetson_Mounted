"""Runtime abort checks (node side) and the PX4 parameters that must be set before flying.

Layering, innermost first:
  1. plan envelope (maneuvers.py):        z <= 2.5 m, |x|,|y| <= 4.5 m
  2. node abort -> AUTO.LAND (this file):  z > max_alt (<= 2.8 m), box, tilt, tracking, telemetry
  3. PX4 itself, independent of the radio: geofence (GF_MAX_VER_DIST <= 3 m) -> Land,
     offboard loss (COM_OF_LOSS_T) -> Land, RC loss -> Land, low battery -> Land.
Every failsafe action is Land, never Return: Return climbs to RTL_RETURN_ALT (60 m by default).
"""
import math
from dataclasses import dataclass

HARD_MAX_ALT = 3.0   # m above takeoff ground; nothing in this package may raise it


@dataclass
class Limits:
    max_alt: float = 2.8          # m above the takeoff ground
    box_xy: float = 6.0           # m from the takeoff point, per axis
    max_track_err: float = 2.5    # m, position segments only (> the largest step, 1.5 m)
    max_tilt_deg: float = 35.0
    telem_timeout: float = 1.0    # s without LOCAL_POSITION_NED / ATTITUDE (radio drops packets)

    def __post_init__(self):
        self.max_alt = min(float(self.max_alt), HARD_MAX_ALT)


def check(lim, pos, roll, pitch, telem_age, sp=None):
    """Return an abort reason or None. pos = measured (x, y, z) in the test frame."""
    if telem_age > lim.telem_timeout:
        return f'telemetry stale {telem_age:.2f}s'
    x, y, z = pos
    if z > lim.max_alt:
        return f'altitude {z:.2f} m > {lim.max_alt:.2f} m'
    if abs(x) > lim.box_xy or abs(y) > lim.box_xy:
        return f'outside box ({x:.2f}, {y:.2f}) > {lim.box_xy:.1f} m'
    tilt = math.degrees(math.acos(max(-1.0, min(1.0, math.cos(roll) * math.cos(pitch)))))
    if tilt > lim.max_tilt_deg:
        return f'tilt {tilt:.1f} deg > {lim.max_tilt_deg:.0f} deg'
    if sp is not None and not sp.xy_velocity_mode:
        err = math.dist(pos, (sp.x, sp.y, sp.z))
        if err > lim.max_track_err:
            return f'tracking error {err:.2f} m > {lim.max_track_err:.1f} m'
    return None


# PX4 v1.16.2 parameter requirements (enum values checked against the v1.16.2 sources).
# name -> (predicate, human-readable requirement)
REQUIRED_PARAMS = {
    'COM_OBL_RC_ACT': (lambda v: v == 4, '4 (Land) on offboard loss'),
    'COM_OF_LOSS_T': (lambda v: 0 < v <= 1.0, '<= 1.0 s offboard-loss timeout'),
    'GF_ACTION': (lambda v: v == 5, '5 (Land) on geofence breach'),
    'GF_MAX_VER_DIST': (lambda v: 0 < v <= HARD_MAX_ALT, f'0 < x <= {HARD_MAX_ALT} m'),
    'GF_MAX_HOR_DIST': (lambda v: 0 < v <= 20, '0 < x <= 20 m'),
    'NAV_RCL_ACT': (lambda v: v == 3, '3 (Land) on RC loss'),
    'COM_LOW_BAT_ACT': (lambda v: v == 2, '2 (Land) on low battery'),
}
# Not blocking, but printed as warnings.
RECOMMENDED_PARAMS = {
    'SDLOG_PROFILE': (lambda v: (int(v) & 0b11001) == 0b11001,
                      'bits 0+3+4 (default + system identification + high rate) = 25'),
    'NAV_DLL_ACT': (lambda v: v in (0, 3), '0 (offboard loss covers it) or 3 (Land)'),
    'RTL_RETURN_ALT': (lambda v: v <= HARD_MAX_ALT,
                       f'<= {HARD_MAX_ALT} m in case a pilot selects Return'),
    'MC_BAT_SCALE_EN': (lambda v: v == 0,
                        '0, otherwise motor commands are pre-scaled by battery voltage and the '
                        'thrust-voltage fit must undo it'),
}


def review_params(values):
    """values: name -> value (None if not received). Returns (errors, warnings)."""
    errors, warnings = [], []
    for table, out in ((REQUIRED_PARAMS, errors), (RECOMMENDED_PARAMS, warnings)):
        for name, (ok, need) in table.items():
            v = values.get(name)
            if v is None:
                out.append(f'{name}: not received (need {need})')
            elif not ok(v):
                out.append(f'{name}={v:g}: need {need}')
    return errors, warnings
