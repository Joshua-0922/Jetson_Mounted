# Jetson_Mounted

Jetson(컴패니언 컴퓨터)에서 Pixhawk(PX4)를 offboard 속도 명령으로 제어하는 **배포 전용** 저장소입니다.
학습·평가 코드와 연구 기록은 `Drone-Bombard-Simulation`에 있고, 여기에는 비행에 필요한 것만 둡니다.

## 구조

| 경로 | 내용 |
|---|---|
| `ros2_ws/src/drop_policy` | 정책 추론 + 미션 상태기계 ROS 2 노드 (numpy 전용, torch 불필요). 가중치는 `models/drop_policy_models.npz` |
| `ros2_ws/src/drone_controller` | 속도 명령을 PX4 `TrajectorySetpoint`(offboard)로 보내는 노드 |
| `ros2_ws/src/payload_drop` | 지상 PC → 무선(MAVLink) `DO_SET_ACTUATOR`로 FMU PWM 서보(MG90D) 투하, PX4 ulog로 투하 지연(명령 → 출력 → 분리 센서) 단계별 측정 |
| `ros2_ws/src/imx219_camera` | IMX219 CSI 카메라 → `/camera/image_raw` (Argus ISP + 소프트웨어 AWB, `header.stamp` = 센서 SOF 시각) |
| `ros2_ws/src/yolo_detector` | `/camera/image_raw` → YOLO(`models/best.pt`, `x_marker`) → `/yolo/detections`, `/yolo/image_annotated`, `/yolo/timing` |
| `sitl/gazebo_models` | Gazebo 기체·페이로드·월드와 PX4 airframe(`4900_gz_x500_drop`). Jetson 또는 PC에서 SITL 검증할 때 사용 |
| `scripts/` | Jetson 설치(`setup_jetson.sh`)·점검(`verify_jetson.sh`), Pixhawk USB uXRCE-DDS 설정(`px4_usb_xrce.py`), 링크 지연 측정(`measure_link_latency.py`), YOLO 부하(`yolo_load.py`), 카메라→YOLO 지연 측정(`measure_yolo_latency.py`, `argus_sof_probe.cpp`) |
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

## 링크 지연 실측 (USB uXRCE-DDS, YOLO 부하 포함)

2026-10-28 08:22, 지상·시동 꺼짐, Jetson 25W 모드, 조건별 60초. 원자료: `measurements/link_usb_20261028_0822_{A,B,C}*`
(링크 JSON, YOLO JSON, tegrastats 로그). YOLO는 `yolo11n` TensorRT FP16 엔진(640×640)에 예제 영상
(`/usr/src/jetson_multimedia_api/data/Video/sample_outdoor_car_1080p_10fps.h264`, 1080p)을 반복해서 넣었습니다(`scripts/yolo_load.py`).

| 조건 | Jetson 부하 | 편도 Pixhawk→Jetson (보정) p50 / p95 / 최대 | 센서 샘플→수신 p50 / p95 | 명령 왕복 p50 / p95 / 최대 (손실) | 오도메트리 |
|---|---|---|---|---|---|
| A. 무부하 | CPU 10%, GPU 0% | 5.2 / 9.2 / 15.0 ms | 6.1 / 10.0 ms | 10.5 / 17.0 / 22.1 ms (0/1200) | 100.0 Hz, 간격 p99 12.0 ms |
| B. YOLO 10 fps | CPU 21%, GPU 14% | 5.3 / 9.2 / 14.1 ms | 6.2 / 10.1 ms | 10.9 / 17.3 / 23.0 ms (0/1200) | 100.0 Hz, 간격 p99 12.3 ms |
| C. YOLO 최대(20 fps) | CPU 25%, GPU 32% | 5.3 / 9.3 / 23.0 ms | 6.2 / 10.0 ms | 11.1 / 17.2 / 21.9 ms (0/1200) | 100.0 Hz, 간격 p99 12.3 ms |

YOLO 자체 지연(프레임 취득 → 결과): B p50 48 / p95 76 ms, C p50 54 / p95 69 ms. 추론만은 약 10~14 ms이고,
나머지는 1080p 프레임의 CPU 전처리(9~14 ms)·후처리(6~8 ms)와 Python 처리입니다. 최대 속도에서도 CPU 25%, GPU 32%라서
병목은 하드웨어가 아니라 단일 스레드 Python 파이프라인입니다.

해석:
- **YOLO 부하는 PX4 링크에 영향을 주지 않았습니다.** 세 조건의 편도·왕복·오도메트리 간격이 측정 오차 안에서 같습니다.
- 링크 지연(편도 약 5 ms, p95 약 9 ms)은 정책 주기(20 Hz, 50 ms)보다 한 자릿수 작습니다. 지연 DR 0~1 step이면 충분합니다.
- **인식 지연은 다릅니다.** 위 50~75 ms는 영상 파일을 읽은 시점부터 잰 값입니다. 실제 카메라의 센서 노출부터 재면 약 100~120 ms이고, 정책 입력에는 2~3 step 지연이 필요합니다. 아래 "카메라 → YOLO 인식 지연"을 참고하십시오.
- 명령 왕복에는 PX4 commander의 처리 주기가 들어가므로 실제 Jetson → Pixhawk 전송 지연보다 큽니다.

측정 시 주의(시계):
- PX4는 메시지 타임스탬프를 Jetson 시계로 바꿀 때 **필터로 다듬은 오프셋**을 씁니다. Jetson은 부팅 때 1970년으로 시작했다가 NTP가 시계를 맞추는 동안 천천히 조정(slew)되므로,
  이때 필터가 33~58 ms 늦어져 원시 편도 값(`uplink_ms`)이 38~63 ms로 부풀었습니다. 표의 값은 패킷마다 바로 측정한 오프셋(`observed_offset`)으로 다시 계산한 `uplink_ms_corrected`입니다.
- 인터넷이 없는 비행장에서는 NTP 조정이 없으므로 이 편향이 생기지 않습니다. 현재 노드는 PX4 메시지의 `timestamp`를 제어에 쓰지 않으므로 비행 제어에는 영향이 없습니다.
- 2026-10-28 00:14의 첫 측정(`link_usb_20261028_0014.json`, 편도 p50 3.0 ms)은 이 보정이 없던 값이라 기준값으로 쓰지 않습니다.

측정 방법:

```bash
MicroXRCEAgent serial --dev /dev/pixhawk -b 921600                                          # 터미널 1
~/yolo-venv/bin/python scripts/yolo_load.py --model ~/yolo_models/yolo11n.engine --fps 10    # 터미널 2 (선택: 부하)
python3 scripts/measure_link_latency.py --duration 60 --json measurements/link.json           # 터미널 3
```

TensorRT 엔진 만들기: `cd ~/yolo_models && ~/yolo-venv/bin/yolo export model=yolo11n.pt format=engine half=True imgsz=640`

## 카메라 → YOLO 인식 지연 (IMX219, sim2real 지연 모델)

2026-11-05 23:11, 실내, Jetson 25W 모드, 조건별 60초. 원자료: `measurements/yolo_latency_20261105_2311_{A,B}.json`.
구성: IMX219 센서 모드 2(1920×1080@30) → ISP에서 1280×720으로 축소, `best.pt`(YOLOv8n, PyTorch FP16, imgsz 640), 노드 2개(`imx219_camera`, `yolo_detector`)는 같은 Jetson에서 DDS로 통신.

**시간 기준은 센서 SOF(start-of-frame, 첫 줄이 센서에서 나온 시각)입니다.** `imx219_camera`가 `header.stamp`에 Argus 센서 타임스탬프를 넣습니다.
이 값은 `nvarguscamerasrc show-latency=true`가 프레임마다 남기는 SOF→버퍼 출력 지연 로그에서 복원하며, 시계는 `CLOCK_MONOTONIC`입니다.
따라서 하위 노드 어디서든 `now - header.stamp`가 그대로 촬영 시점부터의 지연입니다.

| 조건 | 결과 주기 | 버린 카메라 프레임 | SOF → 결과 수신 p50 / p95 / p99 / 최대 | 평균 ± 표준편차 | 무작위 시점에서 본 최신 결과의 나이 p50 / p95 / 최대 |
|---|---|---|---|---|---|
| A. 박스 이미지 발행 켬(기본) | 27.7 Hz | 7.9% | 115.1 / 128.2 / 131.2 / 139.8 ms | 113.2 ± 12.5 ms | 132.3 / 155.5 / 186.4 ms |
| **B. 박스 이미지 끔 (비행용)** | **29.6 Hz** | **1.4%** | **103.0 / 113.3 / 121.0 / 141.0 ms** | **102.1 ± 9.8 ms** | **119.6 / 138.9 / 173.3 ms** |

조건 B의 구간별 지연(p50, SOF 기준 누적):

| 구간 | 누적 | 구간 소요 | 측정 방법 |
|---|---|---|---|
| SOF → Argus 프레임 출력 | 36 ms | 36 ms (줄 읽기 20.4 ms + ISP) | `argus_sof_probe.cpp` |
| → nvarguscamerasrc 버퍼 출력 | 43 ms | 7 ms | `show-latency` 로그 |
| → appsink(BGR 변환 후) | 51 ms | 8 ms | GStreamer PTS |
| → 카메라 노드 발행 | 65 ms | 14 ms (복사, AWB, cv_bridge, 2.7 MB 발행) | 노드 로그 |
| → 검출 노드 수신 | 74 ms | 9 ms (DDS 전송 + 앞 프레임 추론 대기) | `/yolo/timing` |
| → YOLO 추론 끝, 결과 발행 | 102 ms | 25 ms 추론 + 2 ms | `/yolo/timing` |
| → 다른 노드 수신 | 103 ms | 1 ms | `measure_yolo_latency.py` |

시뮬 지연 모델(조건 B 기준):
- **검출 지연**: SOF 기준 `D ~ N(102, 10²) ms`, [90, 141] ms로 자름. 결과는 약 30 Hz(33.8 ms 간격)로 나오고, 1.4%는 빠집니다.
- **노출 보정**: 장면을 실제로 본 시각(노출 중간)은 SOF보다 이릅니다. 영상 y행(높이 H)에 있는 목표의 지연은 `D + exposure/2 − (y/H)·readout`이며, readout은 20.4 ms입니다.
  실내에서는 노출이 33 ms까지 길어지므로(+16.5 ms) 화면 중앙 기준 약 109 ms입니다. 밝은 실외에서 노출이 1 ms 미만이면 화면 중앙 기준 약 93 ms입니다. 실외 노출은 `argus_sof_probe`로 다시 확인하십시오.
- **정책 입력(20 Hz, 50 ms)**: 정책이 받는 최신 결과의 나이는 약 95~175 ms(p50 120 ms)입니다. 따라서 **지연 DR은 2~3 step**이 적절합니다. 링크 지연(0~1 step)과는 별개로 넣어야 합니다.
- 박스 이미지 발행을 켜면 콜백마다 그리기와 발행이 3~4 ms 늘어납니다. 그러면 검출 노드가 카메라 속도(30 Hz)를 못 따라가서 프레임 대기가 생기고, 지연이 약 12 ms 늘어납니다. 비행에서는 `publish_annotated:=false`를 쓰십시오.

측정 방법:

```bash
ros2 launch yolo_detector yolo.launch.py publish_annotated:=false                     # 터미널 1
python3 scripts/measure_yolo_latency.py --duration 60 --json measurements/yolo.json    # 터미널 2
```

지연을 줄일 여지: TensorRT 엔진(`model:=...engine`)으로 추론 25 ms를 줄일 수 있습니다. 카메라와 검출을 한 프로세스로 합치면 14 + 9 ms 중 상당 부분(2.7 MB 이미지 복사와 전송)이 사라집니다.

## 가중치 갱신

`drop_policy_models.npz`는 시뮬 저장소의 `isaac_lab/_export_deploy.py`로 생성합니다(Isaac 컨테이너에서 torch 필요).
새 가중치를 만들면 이 파일만 교체하고, 골든 테스트가 통과하는지 확인한 뒤 커밋하십시오.
관측 26채널의 순서·정규화와 행동 정의가 바뀌면 `policy_core.py`도 함께 수정해야 합니다.

## 알아둘 점

- `drop_policy/launch/drop_eval.launch.py`와 `drop_policy/README.md`는 Gazebo SITL 컨테이너 경로(`/workspace/...`, `/opt/PX4-Autopilot`)를 전제합니다. Jetson에서는 경로를 조정해야 합니다.
- 실기 투하 명령은 `DO_SET_ACTUATOR`(187) 경로를 쓰는 것으로 결정되어 있으나(`docs/sim2real_firmware_stack.md` §0), 현재 노드는 투하 시 `/drop/payload/drop_cmd`(Empty)만 발행하며, 이는 Gazebo 브리지(DetachableJoint) 전용입니다. 실기용 투하 경로(`VehicleCommand` 187)는 아직 구현되지 않았습니다. 또 미션 노드는 `/drop/payload/odom`, `/drop/drone/odom`(Gazebo 참값)도 구독하므로, 실기에서는 `state_source=px4`로 쓰고 착지 판정 등 페이로드 참값 의존 부분을 점검해야 합니다.
- 프로펠러를 장착한 상태로 처음 실행하지 마십시오. 먼저 프로펠러를 분리하고 offboard 명령과 상태 토픽을 확인하십시오.
