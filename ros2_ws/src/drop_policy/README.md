# drop_policy — Isaac Lab 투하 정책의 ROS 2 배포 패키지 (Gazebo SITL 검증 → 실기)

Isaac Lab에서 학습한 비행 정책 L0(PPO)와 착탄점 잔차 GRU(+EMA)를 PX4 위에서 그대로 돌린다.
정책 출력은 **속도 명령**(`[vx, vy, vz, yaw_rate]`, ENU, 10 Hz)이며 기존 `drone_controller`가 PX4 offboard
`TrajectorySetpoint`로 보낸다(20 Hz LPF α 0.4 = Isaac과 동일).

## 구성
| 파일 | 역할 |
|---|---|
| `drop_policy/policy_core.py` | ROS·torch 없는 numpy 코어: 관측 26채널, 행동 처리(clip·rate limit 0.2·scale 4/3/3/1), 공칭 착탄 예측, GRU 잔차, 릴리즈 게이트, PX4 NED/FRD ↔ ENU/FLU 변환. Isaac `task_env.py`와 줄 단위로 대응 |
| `drop_policy/drop_mission_node.py` | 한 에피소드: CLIMB(접근 방위로 yaw 고정) → CRUISE(handoff 상태 재현) → POLICY(10 Hz 정책, 상태 주기마다 릴리즈 판정, 0.22 s 뒤 투하 명령) → HOLD. 결과 JSON |
| `models/drop_policy_models.npz` | L0 actor와 GRU 가중치 + torch 원본과의 골든 입출력 (`isaac_lab/_export_deploy.py`로 생성) |
| `launch/drop_eval.launch.py` | Gazebo(서버) + PX4 v1.15.4 SITL(airframe 4020_gz_x500_drop) + uXRCE agent + ros_gz_bridge + drone_controller + 미션 노드 |
| `config/gz_bridge.yaml` | 페이로드·기체 참값 오도메트리, 투하 명령(DetachableJoint) |
| `scripts/run_gazebo_eval.py` | 시험 환경(handoff 방위·속도·고도·거리·측방 오프셋·바람)별 에피소드 실행 |
| `test/test_policy_core.py` | numpy 포팅 = torch 원본(행동·GRU 1e-4, 착탄 예측 1e-3), 좌표 변환 부호 |

Gazebo 모델(`gazebo_models/x500_drop`, `payload_drop`, `worlds/drop_world.sdf.in`)과 PX4 airframe
(`gazebo_models/px4_airframes/4020_gz_x500_drop`)은 Isaac 기체와 맞췄다: T/W 2.0(최대 로터 1116 rad/s),
페이로드 0.14 m 아래 장착, 원통 Ø10×6 cm 0.1 kg(항력 0.005), 기체 항력 0.06, 기울기 35°, 하강 3 m/s.

## 실행 (drone-bombard-harmonic 컨테이너)
```bash
cp /workspace/gazebo_models/px4_airframes/4020_gz_x500_drop /opt/PX4-Autopilot/build/px4_sitl_default/etc/init.d-posix/airframes/
source /opt/ros/humble/setup.bash && source /root/ros2_ws/install/setup.bash
cd /workspace/ros2_ws && colcon build --packages-select drone_controller drop_policy --symlink-install && source install/setup.bash
python3 -m pytest src/drop_policy/test -q
python3 src/drop_policy/scripts/run_gazebo_eval.py --out /tmp/gz_eval            # 6 환경 × L0/GRU
```

## 규약 (실기에서도 그대로)
- 과제 좌표계: ENU, 원점 = 이륙 지점, 고도 = 기체 원점의 지면 위 높이. PX4 로컬 원점이 달라도 위치 명령은 측정한 PX4 − 상태 오프셋으로 변환한다.
- 기체는 접근 방위를 바라본 채 handoff한다(Isaac 학습 조건). `drone_controller`의 `/drone/cmd/yaw`로 고정.
- 상태 출처 `state_source`: `gz_truth`(SITL 참값, RTK 대용) 또는 `px4`(EKF). PX4 SITL GPS는 수평 0.2 m·수직 0.5 m 잡음이라 RTK보다 나쁘다.
- 릴리즈 지연: 예측기가 평균 0.22 s를 가정하므로 SITL에서는 같은 지연 뒤 분리한다. 실기에서는 서보 실측값으로 바꾼다.
