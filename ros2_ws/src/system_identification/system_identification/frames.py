"""Test frame <-> PX4 local NED.

All maneuvers are written in a "test frame" fixed at the start of the test:
  origin = takeoff point (x, y) and ground height (z = 0 on the ground),
  x = vehicle forward at the start, y = left, z = up  (FLU),
  yaw = heading relative to the start heading, CCW positive.
Using the start heading means an x maneuver is a pure pitch maneuver and a y maneuver is a pure
roll maneuver, which keeps the axes separable in the identification.
"""
import math


def wrap_pi(a):
    return math.atan2(math.sin(a), math.cos(a))


class TestFrame:
    def __init__(self, north0, east0, down_ground, heading0):
        """heading0: PX4 NED yaw at the start (rad, CW from North)."""
        self.n0 = north0
        self.e0 = east0
        self.d0 = down_ground
        self.psi = heading0
        self.c = math.cos(heading0)
        self.s = math.sin(heading0)

    # --- test (FLU) -> NED ---
    def pos_to_ned(self, x, y, z):
        n = self.n0 + x * self.c + y * self.s
        e = self.e0 + x * self.s - y * self.c
        return n, e, self.d0 - z

    def vel_to_ned(self, vx, vy, vz):
        return vx * self.c + vy * self.s, vx * self.s - vy * self.c, -vz

    def yaw_to_ned(self, yaw):
        return wrap_pi(self.psi - yaw)

    # --- NED -> test (FLU) ---
    def pos_from_ned(self, n, e, d):
        dn, de = n - self.n0, e - self.e0
        return dn * self.c + de * self.s, dn * self.s - de * self.c, self.d0 - d

    def vel_from_ned(self, vn, ve, vd):
        return vn * self.c + ve * self.s, vn * self.s - ve * self.c, -vd

    def yaw_from_ned(self, yaw_ned):
        return wrap_pi(self.psi - yaw_ned)
