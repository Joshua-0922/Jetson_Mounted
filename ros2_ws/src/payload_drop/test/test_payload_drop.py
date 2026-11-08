import socket
import struct
import threading
import time

import pytest

from payload_drop import analyze, core
from payload_drop.link import ClockSync, Px4Link

GOOD = {'PWM_AUX_FUNC1': 301, 'PWM_AUX_FUNC2': 2032, 'PWM_MAIN_FUNC1': 101, 'PWM_MAIN_FUNC2': 102,
        'PWM_MAIN_FUNC3': 103, 'PWM_MAIN_FUNC4': 104, 'COM_PREARM_MODE': 2, 'CAM_CAP_FBACK': 1,
        'CAM_CAP_EDGE': 0, 'SDLOG_MODE': 2, 'SDLOG_PROFILE': 1, 'PWM_AUX_REV': 0, 'PWM_MAIN_REV': 0,
        'PWM_AUX_MIN1': 1000, 'PWM_AUX_MAX1': 2000, 'PWM_AUX_DIS1': 1000, 'PWM_AUX_FAIL1': -1,
        'PWM_AUX_TIM0': 50}


# ------------------------------------------------------------------ clock sync
def test_clock_sync_uses_min_rtt_sample():
    c = ClockSync()
    off = 123_456_789
    # symmetric 20 ms round trip, and an asymmetric slow one that would bias the offset
    c.add(1_000_000_000, 1_020_000_000, 1_010_000_000 + off)
    c.add(2_000_000_000, 2_200_000_000, 2_190_000_000 + off)
    o, rtt = c.best()
    assert o == off and rtt == 20_000_000
    t, unc = c.to_px4_us(3_000_000_000)
    assert t == pytest.approx((3_000_000_000 + off) / 1e3) and unc == pytest.approx(10_000)


def test_clock_sync_window_drops_old_samples():
    c = ClockSync(window_s=1.0)
    c.add(0, 1_000_000, 500_000)               # 1 ms rtt, old
    c.add(5_000_000_000, 5_050_000_000, 5_025_000_000 + 7)
    assert c.best() == (7, 50_000_000)


# ------------------------------------------------------------------ config review
def test_pulse_mapping():
    assert core.pulse_us(-1, 1000, 2000) == 1000
    assert core.pulse_us(1, 1000, 2000) == 2000
    assert core.pulse_us(0, 1000, 2000) == 1500
    assert core.pulse_us(1, 1000, 2000, reversed_=True) == 1000


def test_good_config_on_fmu_aux1():
    cfg, err, warn = core.review_config(GOOD, 1, 1.0, -1.0, 'bench')
    assert not err and not warn
    assert (cfg.bank, cfg.channel, cfg.open_us, cfg.close_us) == ('AUX', 1, 2000, 1000)
    assert cfg.capture == [('AUX', 2)]


def test_config_problems():
    p = dict(GOOD, PWM_AUX_FUNC1=0, PWM_MAIN_FUNC5=301, PWM_MAIN_MIN5=1000, PWM_MAIN_MAX5=2000,
             PWM_MAIN_DIS5=0, PWM_MAIN_TIM2=400, COM_PREARM_MODE=0, SDLOG_MODE=0)
    cfg, err, warn = core.review_config(p, 1, 1.0, -1.0, 'bench')
    assert cfg.bank == 'MAIN' and cfg.channel == 5
    text = ' '.join(err + warn)
    assert 'COM_PREARM_MODE' in text and 'SDLOG_MODE' in text and 'IO' in text
    assert 'DIS5' in text and 'TIM2' in text
    _, err, _ = core.review_config(dict(GOOD, PWM_AUX_FUNC1=0), 1, 1.0, -1.0, 'manual')
    assert err and 'Actuator Set 1' in err[0]


# ------------------------------------------------------------------ analysis
def _synthetic(n=10, link_ms=40.0, px4_ms=3.0, release_ms=160.0):
    events, cmd_t, cmd_val, trig = [], [], [], []
    t = 10e6
    for k in range(n):
        for kind, val in (('open', 1.0), ('close', -1.0)):
            t += 2e6 + 137e3 * k
            events.append({'seq': len(events), 'kind': kind, 'value': val, 'px4_send_us': t,
                           'sync_unc_us': 15e3})
            cmd_t.append(t + link_ms * 1e3)
            cmd_val.append(val)
            if kind == 'open':
                trig.append(t + (link_ms + px4_ms + release_ms) * 1e3)
    # FMU outputs at 250 Hz; channel 1 = servo, channel 2 = static
    ts = [int(5e6 + 4000 * i) for i in range(int((t + 3e6 - 5e6) / 4000))]
    servo, val = [], 1000.0
    j = 0
    for x in ts:
        while j < len(cmd_t) and x >= cmd_t[j] + px4_ms * 1e3:
            val = 2000.0 if cmd_val[j] > 0 else 1000.0
            j += 1
        servo.append(val)
    io_t = list(range(int(5e6), int(t + 3e6), 2500))             # I/O outputs (ESCs), 400 Hz
    outputs = {0: (io_t, [[1500.0] * len(io_t) for _ in range(4)]),
               1: (ts, [servo, [0.0] * len(ts)])}
    return events, cmd_t, cmd_val, outputs, trig


def test_analysis_recovers_stages():
    events, cmd_t, cmd_val, outputs, trig = _synthetic()
    rows, info = analyze.analyze(events, cmd_t, cmd_val, outputs, trig)
    assert info['output'] == (1, 0) and info['n_matched'] == len(events)
    drops = [r for r in rows if r['kind'] == 'open']
    for r in drops:
        assert r['link_ms'] == pytest.approx(40.0)
        assert 3.0 <= r['px4_ms'] < 3.0 + 4.0 + 1e-6          # quantised by the 4 ms output period
        assert r['px4_ms'] + r['release_ms'] == pytest.approx(163.0)
        assert r['total_ms'] == pytest.approx(203.0)
    assert all(r['release_ms'] is None for r in rows if r['kind'] == 'close')
    events[0]['sends'] = 2                                     # resent: no link / total time
    r0 = analyze.analyze(events, cmd_t, cmd_val, outputs, trig)[0][0]
    assert r0['link_ms'] is None and r0['total_ms'] is None and r0['release_ms'] is not None
    s = analyze.stats([r['total_ms'] for r in drops])
    assert s['n'] == 10 and s['mean'] == pytest.approx(203.0)


def test_match_ignores_wrong_values_and_far_times():
    ev = [{'kind': 'open', 'value': 1.0, 'px4_send_us': 1e6}]
    # too early, wrong value, match
    assert analyze.match_commands(ev, [0.5e6, 1.1e6, 1.2e6], [1.0, -1.0, 1.0]) == [2]
    assert analyze.match_commands(ev, [9e6], [1.0]) == [None]


def test_imu_onset():
    t = [i * 1000 for i in range(3000)]
    a = [-9.81 + (0.02 if i % 2 else -0.02) for i in range(3000)]
    for i in range(1700, 1720):
        a[i] += 2.0
    assert analyze.imu_onset(t, a, 1_500_000) == 1_700_000


# ------------------------------------------------------------------ link + controller vs a fake PX4
class FakePx4(threading.Thread):
    """PX4 stand-in on UDP: heartbeat, PARAM_VALUE (INT32 packed bytewise), TIMESYNC replies on
    a clock with a fixed offset, COMMAND_ACK for DO_SET_ACTUATOR (drops the first one)."""
    OFFSET_NS = 987_654_321_000

    def __init__(self, port, params):
        super().__init__(daemon=True)
        from pymavlink import mavutil
        self.M = mavutil.mavlink
        self.m = mavutil.mavlink_connection(f'udpout:127.0.0.1:{port}', source_system=1, source_component=1)
        self.params = params
        self.stop = False
        self.commands = []
        self.dropped_first = False

    def run(self):
        M = self.M
        t_hb = 0
        while not self.stop:
            if time.monotonic() - t_hb > 0.2:
                t_hb = time.monotonic()
                self.m.mav.heartbeat_send(M.MAV_TYPE_QUADROTOR, M.MAV_AUTOPILOT_PX4, 0, 0, 0)
            msg = self.m.recv_match(blocking=True, timeout=0.05)
            if msg is None:
                continue
            t = msg.get_type()
            if t == 'PARAM_REQUEST_READ':
                name = msg.param_id
                if name in self.params:
                    raw = struct.unpack('<f', struct.pack('<i', int(self.params[name])))[0]
                    self.m.mav.param_value_send(name.encode(), raw, M.MAV_PARAM_TYPE_INT32, 1, 0)
            elif t == 'TIMESYNC' and msg.tc1 == 0:
                self.m.mav.timesync_send(time.monotonic_ns() + self.OFFSET_NS, msg.ts1)
            elif t == 'COMMAND_LONG' and msg.command == 187:
                self.commands.append((time.monotonic_ns() + self.OFFSET_NS, msg.param1, msg.confirmation))
                if not self.dropped_first:
                    self.dropped_first = True
                    continue
                self.m.mav.command_ack_send(187, M.MAV_RESULT_ACCEPTED)


def _free_port():
    s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    s.bind(('127.0.0.1', 0))
    p = s.getsockname()[1]
    s.close()
    return p


def test_link_and_controller_against_fake_px4():
    port = _free_port()
    link = Px4Link(f'udpin:127.0.0.1:{port}', log=lambda s: None)
    fake = FakePx4(port, GOOD)
    fake.start()
    try:
        assert link.wait_autopilot(timeout=5)
        link.start()
        for _ in range(5):
            link.timesync()
            time.sleep(0.05)
        off, rtt = link.clock.best()
        assert abs(off - FakePx4.OFFSET_NS) < 5e6 and rtt < 50e6

        params = link.get_params(core.first_pass_params() + ['PWM_AUX_FUNC3'])
        assert params['PWM_AUX_FUNC1'] == 301 and params['PWM_MAIN_FUNC3'] == 103
        assert params['PWM_AUX_FUNC3'] is None   # not on the fake: None after the retries
        assert params['COM_PREARM_MODE'] == 2

        seen = []
        ctl = core.DropController(link, 1, 1.0, -1.0, ack_timeout=0.3,
                                  on_event=lambda ev, what: seen.append((ev.seq, what)))
        ev = ctl.send('open')
        t_end = time.monotonic() + 3
        while not ctl.idle() and time.monotonic() < t_end:
            ctl.poll()
            time.sleep(0.02)
        assert ctl.idle()
        assert (0, 'resent') in seen and (0, 'ack') in seen     # first command's ACK was dropped
        assert ev.sends == 2 and ev.ack_result == 0
        # the estimated PX4 send time is within a few ms of when the fake PX4 got it
        assert abs(fake.commands[0][0] / 1e3 - ev.px4_send_us) < 5e3
        assert fake.commands[0][1] == 1.0 and fake.commands[1][2] == 1
    finally:
        fake.stop = True
        link.close()
