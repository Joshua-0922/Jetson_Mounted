# payload_drop

The ground PC sends a drop command to the Pixhawk over the telemetry radio (MAVLink). PX4 then moves the MG90S servo on an FMU PWM output and opens the latch.
The package also **measures the drop delay stage by stage** from the PX4 ulog. The Jetson is not used today. On the Jetson, the same command can be sent over uXRCE-DDS as `/fmu/in/vehicle_command`.

```
ground PC  drop_node ── COMMAND_LONG(DO_SET_ACTUATOR) ──radio──►  PX4 mavlink → vehicle_command
   t0 (PC clock → PX4 clock via TIMESYNC)                       t1 (ulog vehicle_command)
                                                                   │ FunctionActuatorSet → pwm_out (FMU, 250 Hz)
                                                                t2 (ulog actuator_outputs: pulse width change)
                                                                   │ PWM frame (50 Hz: 0–20 ms) + servo travel + latch opening
                                                                t3 (ulog camera_trigger: release-sensor edge, timer capture)
```

| Stage | Formula | Meaning |
|---|---|---|
| `link_ms` | t1 − t0 | Radio + MAVLink processing (this path's version of the Jetson USB uplink already measured: one-way 5/9 ms) |
| `px4_ms` | t2 − t1 | PX4 internal. Resolution = output sample period (4 ms) |
| `release_ms` | t3 − t2 | PWM frame + **servo mechanical travel** + latch → sensor. Expected to be the largest stage |
| `cmd_to_release_ms` | t3 − t1 | The **PX4-side drop delay** that doesn't depend on the link. Use this when sending over the Jetson link |
| `total_ms` | t3 − t0 | The whole delay from ground PC send to release |

The error on t0 is shown as `sync_unc_ms` (min TIMESYNC RTT / 2). The radio's up/down asymmetry can't be measured, so the reported value is an upper bound.
The policy (`drop_policy`'s `release_delay`) should get Jetson link (already measured) + `cmd_to_release_ms`, **both mean and spread**.

## Hardware

| Item | Connection | Why |
|---|---|---|
| ESC ×4 | **I/O PWM OUT 1–4** (`PWM_MAIN_*`, 400 Hz) | No DShot support, so they stay on I/O as now |
| MG90S servo | **FMU PWM OUT 1** (`PWM_AUX_*` 1, 50 Hz) | FMU skips the extra hop through the IO processor. The timer group (AUX1–4) is separate from the ESCs, so it can use its own 50 Hz rate |
| Servo power | **5 V BEC** into the servo rail (+, −). Common ground with the Pixhawk | The Pixhawk does not power the PWM rail. The MG90S draws hundreds of mA when stalled |
| Release sensor (optional, recommended) | Photo-interrupter (slot type, digital output) → **FMU PWM OUT 5** signal pin | Its edge is timestamped by a hardware timer into the ulog (`camera_trigger`) |

- Put the release sensor in a **different timer group** from the servo (AUX1–4 = TIM1, AUX5–6 = TIM4, from the `fmu-v6c` source). If AUX5 isn't brought out on the 6C Mini, use a channel in another group.
- FMU signal pins are 3.3 V. Use a sensor output that is open-collector with a pull-up to 3.3 V, or a 3.3 V output.
  A microswitch bounces. If more than 100 edges arrive within 200 µs, PX4 shuts capture off, so prefer a photo-interrupter.
- Where the sensor points sets what you measure. Pointed at the **payload**: actual separation (the real number for the policy). Pointed at the **servo horn/latch**: servo travel only (bench).

## PX4 settings (QGC)

**Actuators** screen:
- FMU PWM AUX1: function `Peripheral via Actuator Set 1` (=301), Timer group 1–4: `PWM 50 Hz`
- AUX1 `Disarmed` = **closed pulse**, `Min`/`Max` = closed/open pulses
  (the node maps `close_value=-1` → Min and `open_value=+1` → Max; with Rev on, they swap).
  **Disarmed is critical**: PX4 outputs this value before the first command and after disarm. If it isn't the closed position, the latch can open; at 0 (no pulse) the servo goes limp.
- (Sensor) AUX5: function `Camera Capture` (=2032), then `CAM_CAP_FBACK=1`, `CAM_CAP_EDGE` (0 falling / 1 rising), **reboot**

| Parameter | Bench (props off, disarmed) | Flight |
|---|---|---|
| `COM_PREARM_MODE` | **2** (Always): servo works disarmed, motors stay off | 0 (default) |
| `SDLOG_MODE` | **2** (boot..shutdown): logs while disarmed too | 0 or 2 |

On startup the node reads these, reports any problem, and refuses to start if a required one is wrong (`skip_param_check:=true` for SITL only).

**Log rate:** by default PX4 logs `actuator_outputs` every 100 ms, which can't resolve t2.
Copy `config/logger_topics.txt` to the SD card as `/etc/logging/logger_topics.txt` (card reader) and reboot.
While that file exists, PX4 logs **only the topics in it** (it includes the key topics for Flight Review). Remove it after the drop tests.

## Install and run (ground PC, Ubuntu 24.04 + ROS 2 Jazzy)

```bash
pip install --user --break-system-packages pymavlink pyulog
cd ros2_ws && colcon build --packages-select payload_drop --symlink-install && source install/setup.bash
python3 -m pytest src/payload_drop/test -q

# sharing the radio with QGC (QGC auto-connects on UDP 14550)
mavlink-routerd -e 127.0.0.1:14550 -e 127.0.0.1:14560 /dev/ttyUSB0:57600
```

**1) Bench (props removed, disarmed)**: open/close repeated N times at random spacing → latency distribution

```bash
ros2 run payload_drop drop --ros-args -p connection:=udpin:127.0.0.1:14560 -p mode:=bench -p bench_cycles:=30
```

**2) Manual (flight or a single test)**: type `drop` / `close` / `q` + Enter in the terminal

```bash
ros2 run payload_drop drop --ros-args -p connection:=udpin:127.0.0.1:14560
# from another node: ros2 topic pub --once /payload_drop/command std_msgs/String "data: drop"
```

**3) Analysis**: after landing, download the ulog in QGC (Analyze Tools > Log Download)

```bash
ros2 run payload_drop drop_latency <log.ulg> ~/drop_logs/drop_bench_<stamp>.json --csv drop.csv
ros2 run payload_drop drop_latency <log.ulg> --imu     # ulog only (e.g. commands sent from QGC) + accelerometer reaction
```

| Parameter | Default | Description |
|---|---|---|
| `connection` / `baud` | `/dev/ttyUSB0` / 57600 | pymavlink connection string |
| `mode` | `manual` | `manual` \| `bench` (disarmed only) |
| `actuator_set` | 1 | Peripheral via Actuator Set N |
| `open_value` / `close_value` | 1.0 / −1.0 | Function value [−1, 1] → Min..Max |
| `close_on_start` | true | Send closed once at startup (a known state) |
| `auto_close_s` | 0 | manual: re-close N s after a drop (0 = stay open) |
| `bench_cycles`, `bench_open_s`, `bench_close_s`, `bench_jitter_s` | 20, 1.5, 2.0, 0.5 | Bench repetition |
| `log_dir` | `~/drop_logs` | `drop_<mode>_<stamp>.jsonl` (live) / `.json` (params, clock sync, events) |

If no ACK comes within 1 s, the node resends the same value once (idempotent). Resent commands are excluded from `link_ms`, `total_ms` and the ACK RTT.

## Measurement procedure

1. **Bench, no payload, sensor on the servo horn**: 30+ cycles → `release_ms` = servo travel + PWM frame. Compare with the datasheet (about 0.1 s/60° at 4.8 V).
2. **Bench, with payload, sensor on the payload**: hold the airframe up so the payload can fall, 20+ drops → the real release delay under load.
3. **Hover flight drop** (manual): check under real vibration and in-air conditions with `--imu` and the sensor. With a radio link, `link_ms` is much larger than over the Jetson USB link.
4. Report `cmd_to_release_ms` as mean ± std and p95, and use them to set `release_delay` and the delay range in Isaac.

Without a sensor, you can only get up to t2. In that case:
- Use the `--imu` accelerometer reaction (in flight, the payload's mass drop shows up as a step in specific force) as a reference.
- Or film the LED/servo and payload with a 240 fps camera.

## Not yet verified

- `Camera Capture` on an FMU pin of the 6C Mini was checked only in the source (`camera_capture` driver, `CONFIG_DRIVERS_CAMERA_CAPTURE=y`). Check on the bench that `camera_trigger` is recorded when the sensor fires.
- The tests use a fake PX4 (UDP): clock sync, ACK loss → resend, param read. Analysis was checked on synthetic data. It has not been run against PX4 SITL or real hardware yet.
