# system_identification

시뮬과 실기의 차이를 줄이기 위한 **SysID 비행 패키지**입니다. 지상 PC가 텔레메트리 무선(TELEM1, MAVLink)으로 Pixhawk에 직접
offboard 설정점을 10 Hz로 보내고, PX4의 ulog(SD)를 분석 원자료로 씁니다. Jetson은 쓰지 않습니다.

- Pixhawk USB는 uXRCE-DDS 전용이므로(`scripts/px4_usb_xrce.py`) 비행 중 지상 PC가 닿을 수 있는 경로는 무선뿐입니다. 그래서 이 패키지는 uXRCE-DDS가 아닌 **MAVLink(pymavlink)**를 씁니다.
- 고속 데이터(IMU, 모터 명령, 각속도)는 무선으로 받지 않고 **PX4 ulog에서 가져옵니다.** 무선으로는 10 Hz 설정점과 안전 감시에 필요한 상태만 주고받습니다.

## 시험 종류

`ros2 run system_identification sysid_plan --test <이름>`으로 비행하지 않고 계획과 시간을 볼 수 있습니다.
모든 시험은 이륙 → (본 시험) → 원점 복귀 → AUTO.LAND 순서입니다. 좌표는 이륙 지점 기준, 이륙 시 기수 방향 x / 왼쪽 y / 위 z입니다.

| 시험 | 내용 | 시간 | 무엇을 식별하나 |
|---|---|---|---|
| `hover` | 30 s 정지비행 | 46 s | 호버 추력(=mg)과 모터 명령, 전압 한 점. 첫 비행 확인용 |
| `vertical` | z 계단(+0.8, −0.6 m) + z chirp(0.1→1.5 Hz, ±0.4 m, 가속도 ≤ 2.5 m/s²) | 81 s | **추력 곡선(모터 명령 → 추력) 전 구간**, 모터 시정수, 수직 닫힌 루프 응답 |
| `horizontal` | x·y 위치 계단(±1.5 m), x·y chirp(0.1→1.0 Hz, 가속도 ≤ 3 m/s²) | 150 s | 위치 루프 주파수 응답, 기울기-가속도 관계, 피치·롤 축 분리 |
| `yaw` | 요 계단 ±45°, ±90° | 56 s | 요 토크 계수/관성(비율, 아래 주의), 요 응답 |
| `velocity` | 수평 속도 계단 x ±1, ±2 m/s, y ±1 m/s (z는 위치 유지) | 87 s | **정책이 실제로 쓰는 "속도 명령 → 실제 속도" 계단응답**(시정수·오버슈트), 항력 |
| `battery` | 40 s 호버 + 작은 z 계단을 반복, `land_cell_v`(기본 3.6 V/셀)에서 착륙 | ≤ 15 min | **추력-배터리 전압 관계**(만충→방전) |
| `full` | hover + vertical + horizontal + yaw + velocity | 6 min | 한 배터리로 전부 |

### 권장 비행 순서

1. **SITL**(PC에서 `make px4_sitl gz_x500`, `connection:=udpin:0.0.0.0:14540`, `skip_param_check:=true`)로 모든 시험을 한 번씩.
2. 실기 `hover`를 `takeoff:=pilot`으로: 조종사가 Position 모드로 이륙·호버 → RC 스위치로 OFFBOARD → 노드가 시험을 시작합니다.
3. `vertical`, `horizontal`, `yaw`, `velocity`를 각각 따로 비행(만충 배터리). 응답이 괜찮으면 `full`.
4. `battery`를 만충 배터리로 1~2회. 전압 축의 데이터를 촘촘하게 얻습니다.
5. 같은 시험을 **2회 이상** 반복합니다. 한 번은 식별, 한 번은 검증(예측 오차)에 씁니다. 바람이 약한 날(< 2 m/s)에 하십시오.

### 평행이동만으로 충분하지 않은 이유

- 평행이동(위치 이동)은 저주파 위치 루프만 흔듭니다. 기울기가 작고 준정적이라 **관성 모멘트, 모터 시정수가 로그에 거의 드러나지 않습니다.**
- 추력 곡선은 호버 주변 한 점이 아니라 **모터 명령 범위 전체**가 필요하므로 수직 가감속(`vertical`)이 필요합니다.
- 전압 의존성은 **같은 동작을 배터리 전 구간에서** 반복해야 나옵니다(`battery`).
- 정책은 속도 명령을 내므로, 시뮬 정렬의 기준은 위치 이동이 아니라 **속도 계단응답**(`velocity`)입니다(`docs/sim2real_controller_alignment.md` 1·5단계).

### 이 패키지로 하지 않는 것: 롤·피치 관성

10 Hz 위치·속도 설정점은 각속도 루프(수 Hz 이상)를 흔들지 못합니다. 무선으로 각속도 명령을 고주파로 보내는 것은 지연·손실 때문에 위험합니다.
I_xx, I_yy는 다음 중 하나로 얻으십시오.
- **PX4 autotune**(`MC_AT_EN=1`, `MC_AT_APPLY=0`이면 게인을 바꾸지 않고 기체 내부에서 각속도 여기 신호만 넣습니다). 로그의 `vehicle_angular_velocity`(각가속도 포함)와 `actuator_motors`로 식별합니다.
- 조종사가 Stabilized/Acro에서 롤·피치 더블릿(짧게 좌-우)을 넣기.
- **바이필러(두 줄) 진자**: 지상에서 I_zz를 정확히 잴 수 있습니다. 비행 로그만으로는 요 축에서 `k_M / I_zz` 비율만 구해지므로(토크 계수와 관성을 분리할 수 없음) I_zz는 진자로 재는 것을 권합니다.

## 안전 장치

비행 전에 노드가 PX4 파라미터를 읽어서 아래 조건이 아니면 **이륙하지 않습니다**(PX4 기본값은 대부분 조건에 맞지 않습니다. 예: RC 손실 시 Return은 60 m까지 상승).

| 파라미터 | 필요 값 | PX4 기본값 |
|---|---|---|
| `COM_OBL_RC_ACT` | 4 (Land): offboard 신호가 끊기면 착륙 | 0 (Position) |
| `COM_OF_LOSS_T` | ≤ 1.0 s | 1.0 |
| `GF_ACTION` | 5 (Land) | 2 (Hold) |
| `GF_MAX_VER_DIST` | 0 < 값 ≤ 3.0 m (권장 3.0) | 0 (꺼짐) |
| `GF_MAX_HOR_DIST` | 0 < 값 ≤ 20 m (권장 10~15) | 0 (꺼짐) |
| `NAV_RCL_ACT` | 3 (Land) | 2 (Return) |
| `COM_LOW_BAT_ACT` | 2 (Land) | 0 (Warning) |

경고만 하는 권장값: `SDLOG_PROFILE=25`(기본 + System identification + High rate: `sensor_combined`, `vehicle_acceleration`,
`vehicle_angular_velocity`, `actuator_motors`, `vehicle_thrust/torque_setpoint`, `esc_status` 고속 기록), `RTL_RETURN_ALT ≤ 3`,
`MC_BAT_SCALE_EN=0`(1이면 모터 명령이 전압으로 미리 보정되어 추력-전압 관계가 가려집니다). enum 값은 PX4 v1.16.2 소스에서 확인했습니다.

비행 중 단계별 보호:

1. 계획 범위: z ≤ 2.5 m, |x|,|y| ≤ 4.5 m(제동 거리 포함). 시작 전에 계획 전체를 검사합니다.
2. 노드 감시(10 Hz) → **AUTO.LAND**: 고도 > `max_alt`(기본 2.8 m, 코드에서 3.0 m 초과 불가), 박스 이탈(6 m), 기울기 > 35°,
   위치 추종 오차 > 2.5 m, 텔레메트리 끊김 > 1 s, 배터리(3 s 평균) < `land_cell_v`, Ctrl+C.
3. PX4 자체(무선과 무관): 지오펜스 3 m → Land, offboard 신호 1 s 끊김 → Land, RC 손실 → Land, 저전압 → Land.
4. 조종사: RC로 모드를 OFFBOARD 밖으로 바꾸면 노드는 즉시 명령을 멈추고 간섭하지 않습니다.

모든 실패 동작은 Return(상승)이나 Terminate(추락)가 아닌 **Land**입니다. 상부의 컴패니언 컴퓨터를 보호하기 위해서입니다.
그래도 RC 조종사가 항상 대기해야 합니다. 노드는 무선이 끊기면 아무것도 할 수 없고, 그때는 PX4 자체 failsafe에 의존합니다.

## 연결

지상 PC에 텔레메트리 무선을 꽂습니다. 시리얼 포트는 프로그램 하나만 열 수 있으므로 QGC와 함께 쓰려면 라우터로 나눕니다.

```bash
# 방법 A: 노드만 (QGC 없이)
ros2 run system_identification sysid --ros-args -p connection:=/dev/ttyUSB0 -p baud:=57600 -p test:=hover

# 방법 B: QGC와 함께 (mavlink-router)
mavlink-routerd -e 127.0.0.1:14550 -e 127.0.0.1:14560 /dev/ttyUSB0:57600   # QGC는 UDP 14550 자동 연결
ros2 run system_identification sysid --ros-args -p connection:=udpin:127.0.0.1:14560 -p test:=hover
```

57600 baud에서 PX4 텔레메트리와 노드 트래픽(설정점 10 Hz ≈ 0.7 kB/s, 상태 10 Hz ≈ 1 kB/s)이 대역(약 5.7 kB/s)을 나눠 씁니다.
QGC에서 MAVLink 통계(손실률)를 확인하고, 손실이 많으면 `SER_TEL1_BAUD`/무선 속도를 올리십시오.

## 설치·실행 (지상 PC, Ubuntu 24.04 + ROS 2 Jazzy)

```bash
pip install --user --break-system-packages pymavlink    # 또는 venv
cd ros2_ws && colcon build --packages-select system_identification --symlink-install
source install/setup.bash
python3 -m pytest src/system_identification/test -q     # 계획 범위·좌표 변환·안전 검사 단위테스트

ros2 run system_identification sysid --ros-args \
  -p connection:=/dev/ttyUSB0 -p test:=vertical -p takeoff:=pilot
```

| 파라미터 | 기본값 | 설명 |
|---|---|---|
| `connection` / `baud` | `/dev/ttyUSB0` / 57600 | pymavlink 연결 문자열(`udpin:0.0.0.0:14540` = SITL) |
| `test` | `hover` | 위 표 |
| `hover_z` | 1.5 | 기준 고도 [1.2, 1.7] m |
| `takeoff` | `auto` | `auto`: 'fly' 입력 후 노드가 OFFBOARD + 시동 + 이륙. `pilot`: 조종사가 이륙 후 OFFBOARD로 전환 |
| `land_cell_v` | 3.6 | 셀당 전압(3 s 평균)이 이 값 아래면 착륙 |
| `battery_cells` | 0 | 0이면 `BAT1_N_CELLS` 사용 |
| `max_alt`, `box_xy`, `max_tilt_deg`, `max_track_err`, `telem_timeout` | 2.8, 6.0, 35, 2.5, 1.0 | 중단 한계 |
| `log_dir` | `~/sysid_logs` | CSV(10 Hz)·JSON(구간 경계, PX4 파라미터, 중단 사유) |
| `skip_param_check` | false | SITL 전용 |

발행 토픽(10 Hz, 프레임 `sysid_test`): `/sysid/setpoint`(명령 점, 속도 구간에서는 x·y = NaN), `/sysid/position`(측정 점), `/sysid/state`.
CSV의 `px4_boot_ms`는 ulog 시각(부팅 후 µs / 1000)과 같은 시계라서 구간을 ulog에 바로 맞출 수 있습니다.

## 분석 방법(다음 단계)

ulog(`SDLOG_PROFILE=25`)에서:

- **추력 모델**: 가속도계 비력 `vehicle_acceleration`(기체 z) × 질량 = 총 추력(수직 구간은 항력이 작음). `actuator_motors`(0~1)와 `battery_status.voltage_v`로
  `T = Σ k_T(V)·[(1−θ)u_i + θu_i²]`를 최소제곱 적합합니다(`θ`는 PX4 `THR_MDL_FAC`와 같은 꼴). `esc_status`에 RPM이 있으면(양방향 DShot) `T = k·ω²`로 더 정확해집니다.
- **모터 시정수**: `vertical` chirp에서 `actuator_motors` → 측정 추력 사이 1차 지연 + 순지연을 적합합니다. 이미 잰 액추에이터 지연과 비교합니다.
- **관성**: `vehicle_angular_velocity.xyz_derivative`(각가속도) = I⁻¹·τ, τ는 위 추력 모델과 `CA_ROTOR*_PX/PY/KM` 기하로 계산합니다(롤·피치는 autotune/더블릿 로그).
- **항력**: `velocity`의 정속 구간에서 기울기 θ와 속도로 `m·g·tanθ = D(v)`를 적합합니다.
- **닫힌 루프 정렬**: `velocity`·`horizontal` 계단응답을 Isaac 제어기에 같은 명령으로 넣어 시정수·오버슈트가 ±10% 안인지 봅니다.
