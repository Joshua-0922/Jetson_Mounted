"""Drop commands, ACK bookkeeping and PX4 output-configuration checks (no ROS here, unit-testable).

Hardware this is written for: Pixhawk 6C Mini, ESCs on I/O PWM OUT (PWM_MAIN_*), MG90S servo on
FMU PWM OUT (PWM_AUX_*), output function 'Peripheral via Actuator Set N' (= 300 + N).
"""
import threading
import time
from dataclasses import asdict, dataclass, field
from typing import Optional

FUNC_ACTUATOR_SET_BASE = 300          # Peripheral_via_Actuator_Set1 = 301 (output_functions.yaml)
FUNC_CAMERA_CAPTURE = 2032
# Timer groups (outputs in one group share the PWM rate). FMU: boards/px4/fmu-v6c timer_config.cpp
# (TIM1 = 1-4, TIM4 = 5-6, TIM5 = 7-8). I/O: PX4IO (1-2, 3-4, 5-8).
TIMER_GROUP = {
    'AUX': {1: 0, 2: 0, 3: 0, 4: 0, 5: 1, 6: 1, 7: 2, 8: 2},
    'MAIN': {1: 0, 2: 0, 3: 1, 4: 1, 5: 2, 6: 2, 7: 2, 8: 2},
}
SDLOG_MODES = {-1: 'disabled', 0: 'armed..disarm', 1: 'boot..disarm', 2: 'boot..shutdown',
               3: 'RC AUX1', 4: '1st arm..shutdown'}


def first_pass_params():
    names = [f'PWM_{bank}_FUNC{ch}' for bank in ('AUX', 'MAIN') for ch in range(1, 9)]
    return names + ['COM_PREARM_MODE', 'CAM_CAP_FBACK', 'CAM_CAP_EDGE', 'SDLOG_MODE',
                    'SDLOG_PROFILE', 'PWM_AUX_REV', 'PWM_MAIN_REV']


def find_outputs(params, function):
    """[(bank, channel)] whose PWM_<bank>_FUNC<ch> == function."""
    return [(bank, ch) for bank in ('AUX', 'MAIN') for ch in range(1, 9)
            if params.get(f'PWM_{bank}_FUNC{ch}') == function]


def channel_params(bank, ch):
    return [f'PWM_{bank}_{k}{ch}' for k in ('MIN', 'MAX', 'DIS', 'FAIL')] + \
        [f'PWM_{bank}_TIM{TIMER_GROUP[bank][ch]}']


def pulse_us(value, pmin, pmax, reversed_=False):
    """PX4 maps a function value in [-1, 1] linearly to [MIN, MAX] (swapped if reversed)."""
    v = max(-1.0, min(1.0, value))
    if reversed_:
        v = -v
    return pmin + (v + 1.0) / 2.0 * (pmax - pmin)


@dataclass
class OutputConfig:
    bank: str = ''
    channel: int = 0
    pmin: float = 0.0
    pmax: float = 0.0
    disarmed: float = 0.0
    failsafe: float = 0.0
    rate_hz: float = 0.0
    reversed: bool = False
    open_us: float = 0.0
    close_us: float = 0.0
    capture: list = field(default_factory=list)      # [(bank, ch)] with Camera_Capture


def review_config(params, actuator_set, open_value, close_value, mode):
    """params: everything from first_pass_params() + channel_params() of the servo output.
    Returns (cfg or None, errors, warnings)."""
    errors, warnings = [], []
    func = FUNC_ACTUATOR_SET_BASE + actuator_set
    outs = find_outputs(params, func)
    if not outs:
        errors.append(f"no output has function {func} ('Peripheral via Actuator Set {actuator_set}'): "
                      'QGC > Vehicle Setup > Actuators, FMU PWM AUX1 (recommended)')
        return None, errors, warnings
    if len(outs) > 1:
        warnings.append(f'Actuator Set {actuator_set} drives several outputs: {outs} (using the first)')
    bank, ch = outs[0]
    g = lambda k: params.get(f'PWM_{bank}_{k}')      # noqa: E731
    rev_mask = int(params.get(f'PWM_{bank}_REV') or 0)
    cfg = OutputConfig(bank=bank, channel=ch, pmin=g(f'MIN{ch}') or 0, pmax=g(f'MAX{ch}') or 0,
                       disarmed=g(f'DIS{ch}') or 0, failsafe=g(f'FAIL{ch}') if g(f'FAIL{ch}') is not None else -1,
                       rate_hz=g(f'TIM{TIMER_GROUP[bank][ch]}') or 0,
                       reversed=bool(rev_mask & (1 << (ch - 1))),
                       capture=find_outputs(params, FUNC_CAMERA_CAPTURE))
    cfg.open_us = pulse_us(open_value, cfg.pmin, cfg.pmax, cfg.reversed)
    cfg.close_us = pulse_us(close_value, cfg.pmin, cfg.pmax, cfg.reversed)

    if bank == 'MAIN':
        warnings.append(f'servo on I/O MAIN{ch}: the command takes an extra hop through the IO '
                        'processor; FMU PWM (AUX) is the lower-latency path')
    if cfg.rate_hz != 50:
        warnings.append(f'PWM_{bank}_TIM{TIMER_GROUP[bank][ch]}={cfg.rate_hz:g}: MG90S is an analog '
                        'servo, use 50 Hz (outputs in the same timer group share this rate)')
    if not (cfg.pmin < cfg.pmax):
        errors.append(f'PWM_{bank}_MIN{ch}/MAX{ch} = {cfg.pmin:g}/{cfg.pmax:g}: need MIN < MAX')
    if abs(cfg.disarmed - cfg.close_us) > 50:
        warnings.append(f'PWM_{bank}_DIS{ch}={cfg.disarmed:g} us but "closed" is {cfg.close_us:.0f} us: '
                        'before the first command and after disarm PX4 outputs DIS, set it to the '
                        'closed pulse or the latch may open / go limp (0 = no pulse)')
    if cfg.failsafe >= 0 and abs(cfg.failsafe - cfg.close_us) > 50:
        warnings.append(f'PWM_{bank}_FAIL{ch}={cfg.failsafe:g} us differs from closed '
                        f'{cfg.close_us:.0f} us')

    prearm = params.get('COM_PREARM_MODE')
    if mode == 'bench' and not prearm:
        errors.append('bench mode runs disarmed: set COM_PREARM_MODE=2 (Always) so the servo output '
                      'is active while the motors stay off (set it back to 0 after the test)')
    elif not prearm:
        warnings.append('COM_PREARM_MODE=0: the servo only moves while ARMED')
    sdlog = params.get('SDLOG_MODE')
    if mode == 'bench' and sdlog not in (1, 2):
        errors.append(f'SDLOG_MODE={sdlog} ({SDLOG_MODES.get(sdlog, "?")}): bench runs are disarmed, '
                      'set SDLOG_MODE=2 (boot..shutdown) so the ulog is recorded')
    if not cfg.capture:
        warnings.append('no release sensor (no output with function Camera Capture = 2032): '
                        'the analysis stops at the PX4 output; see README')
    elif not params.get('CAM_CAP_FBACK'):
        errors.append(f'release sensor on {cfg.capture[0]} but CAM_CAP_FBACK=0: set 1 and reboot')
    return cfg, errors, warnings


@dataclass
class Event:
    seq: int
    kind: str                          # 'open' (drop) | 'close'
    value: float
    t_send_ns: int                     # PC monotonic
    t_send_wall: float
    px4_send_us: Optional[float]       # send time on the PX4 boot clock (TIMESYNC estimate)
    sync_unc_us: Optional[float]       # +/- bound of that estimate (min RTT / 2)
    armed: bool
    landed_state: int
    main_mode: int
    sends: int = 1
    t_ack_ns: Optional[int] = None
    ack_result: Optional[int] = None

    @property
    def ack_rtt_ms(self):
        # after a resend it is unknown which copy was acknowledged
        if self.t_ack_ns is None or self.sends > 1:
            return None
        return (self.t_ack_ns - self.t_send_ns) / 1e6

    def as_dict(self):
        d = asdict(self)
        d['ack_rtt_ms'] = self.ack_rtt_ms
        return d


class DropController:
    """Sends open/close, matches COMMAND_ACKs (FIFO), resends once if no ACK."""

    def __init__(self, link, actuator_set, open_value, close_value, ack_timeout=1.0, on_event=None):
        self.link = link
        self.actuator_set = actuator_set
        self.values = {'open': open_value, 'close': close_value}
        self.ack_timeout_ns = int(ack_timeout * 1e9)
        self.events = []
        self._pending = []
        self._lock = threading.Lock()        # ACKs arrive on the MAVLink rx thread
        self.on_event = on_event or (lambda ev, what: None)
        link.on_ack = self._ack

    def send(self, kind):
        st = self.link.snapshot()
        t_ns = self.link.set_actuator(self.actuator_set, self.values[kind])
        px4_us, unc = self.link.clock.to_px4_us(t_ns)
        ev = Event(len(self.events), kind, self.values[kind], t_ns, time.time(), px4_us, unc,
                   st.armed, st.landed_state, st.main_mode)
        with self._lock:
            self.events.append(ev)
            self._pending.append(ev)
        self.on_event(ev, 'sent')
        return ev

    def _ack(self, t_ns, result):
        # The ACK of a resend may arrive after we gave up on the first one: give it to the oldest.
        with self._lock:
            if not self._pending:
                return
            ev = self._pending.pop(0)
            ev.t_ack_ns, ev.ack_result = t_ns, result
        self.on_event(ev, 'ack')

    def poll(self):
        now = time.monotonic_ns()
        out = []
        with self._lock:
            for ev in list(self._pending):
                if now - ev.t_send_ns <= ev.sends * self.ack_timeout_ns:
                    continue
                if ev.sends >= 2:
                    self._pending.remove(ev)
                    out.append((ev, 'no_ack'))
                else:
                    # same value again: idempotent. The analysis uses the FIRST arrival in the ulog.
                    self.link.set_actuator(self.actuator_set, ev.value, confirmation=1)
                    ev.sends += 1
                    out.append((ev, 'resent'))
        for ev, what in out:
            self.on_event(ev, what)

    def idle(self):
        with self._lock:
            return not self._pending
