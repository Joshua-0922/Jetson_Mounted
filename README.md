# Jetson_Mounted

Jetson(컴패니언 컴퓨터)에서 Pixhawk(PX4)를 offboard 속도 명령으로 제어하는 **배포 전용** 저장소입니다.
학습·평가 코드와 연구 기록은 `Drone-Bombard-Simulation`에 있고, 여기에는 비행에 필요한 것만 둡니다.

## 구조

| 경로 | 내용 |
|---|---|
| `ros2_ws/src/drop_policy` | 정책 추론 + 미션 상태기계 ROS 2 노드 (numpy 전용, torch 불필요). 가중치는 `models/drop_policy_models.npz` |
| `ros2_ws/src/drone_controller` | 속도 명령을 PX4 `TrajectorySetpoint`(offboard)로 보내는 노드 |
| `sitl/gazebo_models` | Gazebo 기체·페이로드·월드와 PX4 airframe(`4900_gz_x500_drop`). Jetson 또는 PC에서 SITL 검증할 때 사용 |
| `scripts/` | Jetson 설치(`setup_jetson.sh`)·점검(`verify_jetson.sh`), Pixhawk USB uXRCE-DDS 설정(`px4_usb_xrce.py`), 링크 지연 측정(`measure_link_latency.py`) |
| `measurements/` | 실측 원자료(JSON) |
| `docs/` | 배포 스택 결정(`sim2real_firmware_stack`)과 시뮬 제어기 정렬(`sim2real_controller_alignment`) |

## 하드웨어·소프트웨어 구성 (2026-10-28 기준)

| 항목 | 버전 |
|---|---|
| 비행 제어기 | Holybro Pixhawk 6C Mini (FMU v6C), **PX4 v1.16.2** |
| 컴패니언 | Jetson Orin Nano Super, JetPack 7.2.1 (Ubuntu 24.04, CUDA 13.2, TensorRT 10.16) |
| ROS 2 | **Jazzy** (Ubuntu 24.04에서는 Humble을 쓸 수 없음) |
| `px4_msgs` | `release/1.16` |
| DDS 브리지 | Micro XRCE-DDS Agent v2.4.2 (Fast-DDS v2.12.2) |
| 연결 | **Jetson ↔ Pixhawk: USB-C (uXRCE-DDS)**, QGroundControl ↔ Pixhawk: 텔레메트리 무선(TELEM1, MAVLink 57600) |
| YOLO(선택) | `~/yolo-venv`: PyTorch 2.14.1+cu130, Ultralytics 8.4, numpy 1.26 |

## 설치 (Jetson)

```bash
bash scripts/setup_jetson.sh     # sudo 암호를 묻습니다. 일반 터미널에서 실행
# 로그아웃/로그인(dialout 그룹 반영) 후
bash scripts/verify_jetson.sh
```

`setup_jetson.sh`가 하는 일: ROS 2 Jazzy, Micro XRCE-DDS Agent 빌드·설치, `px4_msgs`(`release/1.16`) + 이 워크스페이스 빌드, YOLO venv,
`dialout` 그룹, udev 규칙(`/dev/pixhawk` 링크 + ModemManager가 Pixhawk 포트에 AT 명령을 보내지 않도록 제외).

Agent 빌드에는 업스트림 문제 두 가지에 대한 우회가 들어 있습니다. Fast-DDS `2.12.x` 브랜치가 삭제되어 `v2.12.2` 태그로 고정했고,
GCC 13에서 `<cstdint>` 누락 오류가 나서 `CXXFLAGS="-include cstdint"`를 줍니다.

### 수동 빌드

`px4_msgs`는 이 저장소에 포함하지 않았습니다. PX4 펌웨어 버전과 같은 브랜치를 받으십시오.

```bash
cd ros2_ws/src
git clone -b release/1.16 https://github.com/PX4/px4_msgs.git
cd .. && source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_msgs drone_controller drop_policy --symlink-install
source install/setup.bash
python3 -m pytest src/drop_policy/test -q      # numpy 포팅이 torch 원본과 일치하는지(골든 테스트)
```

## PX4 v1.16 주의점

- 버전이 1 이상인 메시지는 토픽 이름에 `_v<N>`이 붙습니다. 노드가 쓰는 메시지 중에서는 상태 토픽만 해당하며 **`/fmu/out/vehicle_status_v1`**입니다.
  나머지(`vehicle_odometry`, `vehicle_command`, `vehicle_command_ack`, `trajectory_setpoint`, `offboard_control_mode`)는 이름이 그대로입니다.
- 제어기 기본값(`MPC_*_VEL_P/I/D_ACC`, `MPC_TILTMAX_AIR`, `MPC_Z_VEL_MAX_DN` 등)은 v1.15.4와 같습니다. 비교 내용은 `docs/sim2real_firmware_stack.md` 상단 메모에 있습니다.
- v1.16에서 airframe 4020은 `gz_tiltrotor`라서 SITL airframe을 `4900_gz_x500_drop`으로 바꿨습니다.

## Pixhawk 연결 (USB uXRCE-DDS)

PX4 v1.16은 USB에서 MAVLink·NSH·u-blox만 자동 시작합니다. uXRCE-DDS를 USB에서 쓰려면 Pixhawk를 USB로 연결한 상태에서 한 번 설정합니다.

```bash
~/yolo-venv/bin/python scripts/px4_usb_xrce.py status   # 읽기 전용
~/yolo-venv/bin/python scripts/px4_usb_xrce.py enable   # SD에 etc/extras.txt 작성 + SYS_USB_AUTO=0
```

- `extras.txt` 내용: `uxrce_dds_client start -t serial -d /dev/ttyACM0 -b 921600`. 클라이언트는 포트를 1초마다 다시 열기 때문에 부팅 순서나 케이블 재연결과 무관합니다.
- 이후 USB는 uXRCE-DDS 전용이므로 **QGroundControl은 텔레메트리 무선(TELEM1)으로 연결**합니다.
- 되돌리기: SD의 `extras.txt` 내용을 `param set SYS_USB_AUTO 2`, `param save`로 바꿔 한 번 부팅한 뒤 파일을 지우거나, 텔레메트리 무선으로 QGC에서 `SYS_USB_AUTO=2`로 설정합니다.
- USB가 연결된 상태에서 시동을 걸려면 `CBRK_USB_CHK=197848`이 필요합니다(현재 기체는 이미 이 값입니다). 안전 점검 하나를 끄는 설정임을 알고 쓰십시오.

실행:

```bash
MicroXRCEAgent serial --dev /dev/pixhawk -b 921600      # udev 규칙이 없으면 /dev/ttyACM0
```

## 링크 지연 실측 (USB uXRCE-DDS)

2026-10-28, 지상·시동 꺼짐·무부하(측정 스크립트만 실행), 60초. 원자료: `measurements/link_usb_20261028_0014.json`.

| 항목 | p50 | p95 | p99 | 최대 |
|---|---|---|---|---|
| Pixhawk → Jetson 편도 (`vehicle_odometry` 발행 → ROS 2 수신) | **2.99 ms** | 4.34 | 5.92 | 10.8 |
| 센서 샘플 → ROS 2 수신 (`timestamp_sample` 기준) | 3.67 ms | 5.02 | 6.60 | 11.5 |
| 명령 왕복 (Jetson `VehicleCommand` → PX4 ACK, 1199/1200 응답) | 9.10 ms | 15.5 | 19.4 | 24.7 |
| PX4 timesync 왕복 | 0.74 ms | 0.99 | 1.10 | 1.18 |
| `vehicle_odometry` 수신률 | **98.8 Hz** (간격 p50 10.1 ms, p95 11.6 ms) | | | |

- 편도 값은 PX4가 타임스탬프를 Agent 시계로 옮겨 주는 기능(`UXRCE_DDS_SYNCT=1`)에 기대며, 오차는 timesync 왕복의 절반(약 ±0.4 ms) 이내입니다.
- 명령 왕복에는 PX4 commander의 처리 주기가 들어가므로 실제 Jetson → Pixhawk 전송 지연보다 큽니다.
- 정책 주기(20 Hz, 50 ms)보다 한 자릿수 작으므로 지연 DR 0~1 step이면 충분합니다. 오도메트리가 약 100 Hz로 들어오므로 100 Hz 투하 게이트에도 매 주기 새 값이 있습니다.
- 시동 시 로거 부하, 정책 노드·YOLO 동시 실행 조건에서는 다시 재야 합니다(프로펠러 분리 후).

측정 방법:

```bash
MicroXRCEAgent serial --dev /dev/pixhawk -b 921600                                 # 터미널 1
python3 scripts/measure_link_latency.py --duration 60 --json measurements/link.json  # 터미널 2
```

## 가중치 갱신

`drop_policy_models.npz`는 시뮬 저장소의 `isaac_lab/_export_deploy.py`로 생성합니다(Isaac 컨테이너에서 torch 필요).
새 가중치를 만들면 이 파일만 교체하고, 골든 테스트가 통과하는지 확인한 뒤 커밋하십시오.
관측 26채널의 순서·정규화와 행동 정의가 바뀌면 `policy_core.py`도 함께 수정해야 합니다.

## 알아둘 점

- `drop_policy/launch/drop_eval.launch.py`와 `drop_policy/README.md`는 Gazebo SITL 컨테이너 경로(`/workspace/...`, `/opt/PX4-Autopilot`)를 전제합니다. Jetson에서는 경로를 조정해야 합니다.
- 실기 투하 명령은 `DO_SET_ACTUATOR`(187) 경로를 쓰는 것으로 결정되어 있으나(`docs/sim2real_firmware_stack.md` §0), 현재 노드는 투하 시 `/drop/payload/drop_cmd`(Empty)만 발행하며, 이는 Gazebo 브리지(DetachableJoint) 전용입니다. 실기용 투하 경로(`VehicleCommand` 187)는 아직 구현되지 않았습니다. 또 미션 노드는 `/drop/payload/odom`, `/drop/drone/odom`(Gazebo 참값)도 구독하므로, 실기에서는 `state_source=px4`로 쓰고 착지 판정 등 페이로드 참값 의존 부분을 점검해야 합니다.
- 프로펠러를 장착한 상태로 처음 실행하지 마십시오. 먼저 프로펠러를 분리하고 offboard 명령과 상태 토픽을 확인하십시오.
