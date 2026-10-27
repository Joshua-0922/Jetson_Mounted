#!/usr/bin/env python3
"""Run PX4's uXRCE-DDS client on the USB port (instead of TELEM2), configured over USB MAVLink.

    ~/yolo-venv/bin/python scripts/px4_usb_xrce.py status            # read-only
    ~/yolo-venv/bin/python scripts/px4_usb_xrce.py enable            # write SD extras + SYS_USB_AUTO=0, reboot

PX4 v1.16 auto-starts only MAVLink/NSH/u-blox on USB (cdcacm_autostart), so `enable`:
  1. writes /fs/microsd/etc/extras.txt:  uxrce_dds_client start -t serial -d /dev/ttyACM0 -b 921600
     (the client retries opening the port every 1 s, so boot order and USB re-plug are fine)
  2. sets SYS_USB_AUTO=0 so MAVLink no longer claims /dev/ttyACM0
  3. reboots the FC.
After that, USB carries only uXRCE-DDS: QGroundControl must use a telemetry radio (TELEM1).
Undo: put a microSD extras.txt containing `param set SYS_USB_AUTO 2` + `param save` (or reflash),
then delete extras.txt.

Arming with USB connected is blocked by PX4 ("Flying with USB is not safe") unless
CBRK_USB_CHK=197848. This script never touches that circuit breaker.
"""
import argparse
import struct
import sys
import time

from pymavlink import mavutil

EXTRAS = "/fs/microsd/etc/extras.txt"
XRCE_LINE = "uxrce_dds_client start -t serial -d /dev/ttyACM0 -b 921600"
PARAMS = ["SYS_USB_AUTO", "CBRK_USB_CHK", "UXRCE_DDS_CFG", "UXRCE_DDS_SYNCT", "UXRCE_DDS_DOM_ID", "UXRCE_DDS_KEY"]
SHELL_DEV = 10        # SERIAL_CONTROL_DEV_SHELL
SHELL_FLAGS = 2 | 4 | 16   # RESPOND | EXCLUSIVE | MULTI


class Px4:
    def __init__(self, dev):
        self.m = mavutil.mavlink_connection(dev, baud=115200)
        self.m.wait_heartbeat(timeout=10)

    def version(self):
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     mavutil.mavlink.MAV_CMD_REQUEST_MESSAGE, 0,
                                     mavutil.mavlink.MAVLINK_MSG_ID_AUTOPILOT_VERSION, 0, 0, 0, 0, 0, 0)
        av = self.m.recv_match(type="AUTOPILOT_VERSION", blocking=True, timeout=5)
        if not av:
            return "?"
        v = av.flight_sw_version
        return f"{(v >> 24) & 0xff}.{(v >> 16) & 0xff}.{(v >> 8) & 0xff}"

    def get(self, name):
        for _ in range(3):
            self.m.mav.param_request_read_send(self.m.target_system, self.m.target_component, name.encode(), -1)
            t = time.time()
            while time.time() - t < 1.5:
                r = self.m.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
                if r and r.param_id == name:
                    if r.param_type in (5, 6):   # PX4 packs INT32 bytewise into the float field
                        return struct.unpack("<i", struct.pack("<f", r.param_value))[0]
                    return r.param_value
        return None

    def set_int(self, name, value):
        packed = struct.unpack("<f", struct.pack("<i", value))[0]
        self.m.mav.param_set_send(self.m.target_system, self.m.target_component, name.encode(), packed,
                                  mavutil.mavlink.MAV_PARAM_TYPE_INT32)
        time.sleep(0.3)
        return self.get(name) == value

    def nsh(self, cmd, wait=1.5):
        data = (cmd + "\n").encode()
        for i in range(0, len(data), 70):    # SERIAL_CONTROL carries 70 bytes per message
            chunk = data[i:i + 70]
            self.m.mav.serial_control_send(SHELL_DEV, SHELL_FLAGS, 0, 0, len(chunk), list(chunk.ljust(70, b"\0")))
        out, t = b"", time.time()
        while time.time() - t < wait:
            r = self.m.recv_match(type="SERIAL_CONTROL", blocking=True, timeout=0.2)
            if r and r.count:
                out += bytes(r.data[:r.count])
            else:
                self.m.mav.serial_control_send(SHELL_DEV, SHELL_FLAGS, 0, 0, 0, [0] * 70)
        text = out.decode(errors="replace").replace("\x1b[K", "")
        # drop the echoed command and the trailing prompt
        lines = [ln for ln in text.splitlines() if ln.strip() and not ln.startswith("nsh>") and ln.strip() != cmd]
        return "\n".join(lines)

    def release_shell(self):
        self.m.mav.serial_control_send(SHELL_DEV, 0, 0, 0, 0, [0] * 70)

    def reboot(self):
        self.m.mav.command_long_send(self.m.target_system, self.m.target_component,
                                     mavutil.mavlink.MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN, 0, 1, 0, 0, 0, 0, 0, 0)


def status(fc):
    print(f"PX4 firmware {fc.version()}")
    for p in PARAMS:
        print(f"  {p:18s} {fc.get(p)}")
    fc.nsh("")
    print(f"--- {EXTRAS}:\n{fc.nsh('cat ' + EXTRAS) or '(missing)'}")
    print(f"--- uxrce_dds_client status:\n{fc.nsh('uxrce_dds_client status')}")
    fc.release_shell()


def enable(fc, force, reboot=True):
    fc.nsh("")
    sd = fc.nsh("ls /fs/microsd")
    if "No such" in sd or "not found" in sd.lower():
        sys.exit("no microSD card mounted: extras.txt needs one")
    current = fc.nsh("cat " + EXTRAS)
    exists = current and "No such file" not in current and "open" not in current.lower()
    if exists and current.strip() != XRCE_LINE and not force:
        sys.exit(f"{EXTRAS} already has other content (use --force to overwrite):\n{current}")
    fc.nsh("mkdir /fs/microsd/etc")
    fc.nsh(f'echo "{XRCE_LINE}" > {EXTRAS}')
    written = fc.nsh("cat " + EXTRAS).strip()
    fc.release_shell()
    if written != XRCE_LINE:
        sys.exit(f"extras.txt write could not be verified, got: {written!r}")
    print(f"wrote {EXTRAS}: {written}")
    if not fc.set_int("SYS_USB_AUTO", 0):
        sys.exit("setting SYS_USB_AUTO=0 failed")
    if not reboot:
        print("SYS_USB_AUTO = 0 (saved); takes effect at the next power-up")
        return
    print("SYS_USB_AUTO = 0; rebooting FC (USB MAVLink will be gone after this)")
    fc.reboot()


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("action", choices=["status", "enable"])
    ap.add_argument("--dev", default="/dev/pixhawk" if __import__("os").path.exists("/dev/pixhawk") else "/dev/ttyACM0")
    ap.add_argument("--force", action="store_true", help="overwrite an existing extras.txt")
    ap.add_argument("--no-reboot", action="store_true", help="only write the config; applies at next power-up")
    a = ap.parse_args()
    fc = Px4(a.dev)
    status(fc) if a.action == "status" else enable(fc, a.force, reboot=not a.no_reboot)


if __name__ == "__main__":
    main()
