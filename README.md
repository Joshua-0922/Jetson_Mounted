# Jetson_Mounted

Jetson(컴패니언 컴퓨터)에서 Pixhawk(PX4)를 offboard 속도 명령으로 제어하는 **배포 전용** 저장소입니다.
학습·평가 코드와 연구 기록은 `Drone-Bombard-Simulation`에 있고, 여기에는 비행에 필요한 것만 둡니다.

## 구조

| 경로 | 내용 |
|---|---|
| `ros2_ws/src/drop_policy` | 정책 추론 + 미션 상태기계 ROS 2 노드 (numpy 전용, torch 불필요). 가중치는 `models/drop_policy_models.npz` |
| `ros2_ws/src/drone_controller` | 속도 명령을 PX4 `TrajectorySetpoint`(offboard)로 보내는 노드 |
| `sitl/gazebo_models` | Gazebo 기체·페이로드·월드와 PX4 airframe(`4020_gz_x500_drop`). Jetson 또는 PC에서 SITL 검증할 때 사용 |
| `docs/` | 배포 스택 결정(`sim2real_firmware_stack`)과 시뮬 제어기 정렬(`sim2real_controller_alignment`) |

## 빌드 (ROS 2 Humble)

`px4_msgs`는 이 저장소에 포함하지 않았습니다. PX4 펌웨어 버전(v1.15.x)과 맞는 브랜치를 직접 받으십시오.

```bash
cd ros2_ws/src
git clone -b release/1.15 https://github.com/PX4/px4_msgs.git
cd .. && source /opt/ros/humble/setup.bash
colcon build --packages-select px4_msgs drone_controller drop_policy --symlink-install
source install/setup.bash
python3 -m pytest src/drop_policy/test -q      # numpy 포팅이 torch 원본과 일치하는지(골든 테스트)
```

## 가중치 갱신

`drop_policy_models.npz`는 시뮬 저장소의 `isaac_lab/_export_deploy.py`로 생성합니다(Isaac 컨테이너에서 torch 필요).
새 가중치를 만들면 이 파일만 교체하고, 골든 테스트가 통과하는지 확인한 뒤 커밋하십시오.
관측 26채널의 순서·정규화와 행동 정의가 바뀌면 `policy_core.py`도 함께 수정해야 합니다.

## 알아둘 점

- `drop_policy/launch/drop_eval.launch.py`와 `README.md`는 Gazebo SITL 컨테이너 경로(`/workspace/...`, `/opt/PX4-Autopilot`)를 전제합니다. Jetson에서는 경로를 조정해야 합니다.
- 실기 투하 명령은 `DO_SET_ACTUATOR`(187) 경로를 쓰는 것으로 결정되어 있으나(`docs/sim2real_firmware_stack.md` §0), 현재 노드는 투하 시 `/drop/payload/drop_cmd`(Empty)만 발행하며, 이는 Gazebo 브리지(DetachableJoint) 전용입니다. 실기용 투하 경로(`VehicleCommand` 187)는 아직 구현되지 않았습니다. 또 미션 노드는 `/drop/payload/odom`, `/drop/drone/odom`(Gazebo 참값)도 구독하므로, 실기에서는 `state_source=px4`로 쓰고 착지 판정 등 페이로드 참값 의존 부분을 점검해야 합니다.
- 프로펠러를 장착한 상태로 처음 실행하지 마십시오. 먼저 프로펠러를 분리하고 offboard 명령과 상태 토픽을 확인하십시오.
