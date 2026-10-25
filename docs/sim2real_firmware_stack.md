---
date: 2026-10-03
tags: [research, sim2real, hardware, px4, experiment-design]
status: active
type: research
---

> 2026-10-03 다중 에이전트 조사(하드웨어·선행연구·펌웨어 1차 조사 → 상호 결과 공유·수정 + 스케일 판단 → 종합 → 비판 검토 → 최종본)의 산출물입니다.
> 관련: [[research/sim2real_strategy]] · [[research/sim2real_hardware_spec]] · [[research/sim2real_prior_hw_experiments]] · [[research/sim2real_scale_wind_retrain]] · [[research/sim2real_strategy_review]] · [[research/sim2real_gap]] · [[research/perception_integration_survey]] · [[00_index]]

# sim2real (4) 펌웨어·배포 스택 — PX4 offboard 속도, ROS 2 노드, 관측 26채널 출처, SITL

## 교차 검토로 바뀐 점

| # | 바뀐 점 | 원인 | 무엇이 바뀌었나 |
|---|---|---|---|
| 1 | **실기 기체가 2.07 kg이 아니라 X500 V2급 약 1.3~1.5 kg(추정)으로 정해짐** | (1) 결과: 4종 범위(≤2 kg) 때문에 시뮬 기체를 그대로 복제할 수 없다. 권장안은 X500 V2 + Pixhawk 6C, T/W 약 3.5 이상(추정) | 1차본에서는 "2 kg 미만 기체를 만들 때만 L0 미세조정"이라는 조건부 항목이었다. 이제는 **기본 경로**다. 질량·관성·T/W가 모두 바뀌므로 Isaac 기체 파라미터 갱신, 속도 루프 응답 재정렬, GRU 재적합을 사실상 필수로 둔다(§6). |
| 2 | **질량보다 "닫힌 루프 속도 응답"을 맞추는 쪽이 핵심이라고 정리함** | (2) 결과: DCE(ICRA 2024)는 우리와 같은 구조(속도 + 요레이트 10 Hz, 오토파일럿 속도 루프)에서 **오토파일럿 속도 컨트롤러의 계단응답 시정수를 ±10% 랜덤화**해 zero-shot에 성공했다. Learning to Throw Table V에서는 저수준 컨트롤러를 이상화하면 오차가 3.7배가 된다 | 1차본의 방안 B(시뮬에 PID 이식)와 C(시스템 식별)를 하나로 묶었다. **실기·SITL 계단응답으로 Isaac 속도 루프를 맞추고, 시정수를 ±10% DR**하는 것을 본선으로 정했다. PX4는 HTE와 z 적분으로 질량을 흡수하므로, 속도 수준 정책에는 질량 자체보다 닫힌 루프 응답이 직접 보인다. 이 부분은 근거를 둔 추정이다. |
| 3 | **투하 기구 명령 경로의 불일치를 해소함** | (1)은 `PD_GRIPPER_EN` / `MAV_CMD_DO_GRIPPER`를 권장했다 | 로컬 v1.15.4 소스를 다시 확인했다. `CONFIG_MODULES_PAYLOAD_DELIVERER`는 `boards/px4/sitl/default.px4board`에만 있고, **fmu-v6c와 fmu-v6x 보드 설정에는 없다**. Pixhawk 6C 기본 펌웨어에서는 gripper 경로가 동작하지 않는다. **`DO_SET_ACTUATOR`(187) + "Peripheral via Actuator Set"을 그대로 채택**하고 (1)과 합의 사항으로 둔다. |
| 4 | **Learning to Throw의 펌웨어** | (2) 결과: Betaflight, CTBR 50 Hz, 컨트롤러를 실측 동정해 시뮬에 재현 | 1차본의 "펌웨어 이름 미확인"을 **Betaflight**로 고쳤다((2)의 확인 결과 인용). CTBR 계열이 2 kg급·T/W 2 체제와 맞지 않는다는 판단은 그대로다. 비교 근거로 0.21 kg, T/W 6.8을 덧붙였다. |
| 5 | **PX4 + 컴패니언 + 속도 정책 조합의 선례를 보강함** | (2): Neural-Fly(2.6 kg, T/W 2.2, Pixhawk 4/PX4 + RPi 4), Ishihara(PX4 캐스케이드 PID 유지 + 10 Hz 잔차), RESC와 Narrow Gaps(Gazebo + PX4 SITL sim-to-sim) | 컴패니언 컴퓨터를 **Raspberry Pi 5**로 확정했다((1)과 일치). Gazebo + PX4 SITL 단계의 목적을 선행연구 근거에 맞춰 "인터페이스·지연·좌표계 검증"으로 한정했다. |
| 6 | **측위와 고도 기준** | (1): 실외 RTK F9P, 실내 모캡 | 관측 5·16·18번(고도, 낙하시간)의 출처를 **RTK 높이(EKF2 GPS 높이 기준)**로 정했다. 실내에서는 모캡을 외부 비전으로 융합한다. baro 드리프트 문제는 이 결정으로 해소된다. |
| 7 | **운용 풍속 상한을 안전 감시에 반영함** | (1): 바람 상한 7.5 m/s에서는 기울기 35°가 거의 포화되므로 평균풍 ≤ 5 m/s 운용 상한을 권고(근거 실험 없음) | 이륙 전 풍속계 값으로 비행 여부를 판정하는 Go/No-go 규칙을 추가했다. 정책 입력에는 넣지 않는다. |
| 8 | **Gazebo 기체 모델 갱신** | (1)의 기체 변경 | 기존 `x500_bombard/model.sdf`의 본체 2.0 kg을 **실측 X500 질량·관성으로 갱신**해야 한다. 1차본은 기존 모델을 그대로 재사용한다고 가정했다. |
| 9 | **odometry 발행률은 실측 대상으로 남김** | 배포 스택 자체 점검 | v1.15.4 `dds_topics.yaml`에서 `/fmu/out/vehicle_odometry`와 `vehicle_status`에는 `rate_limit` 항목이 없다. 실제 수신 Hz는 이 파일만으로 판단할 수 없어 **미확인**이다. 100 Hz 투하 게이트를 쓰려면 이 값을 `ros2 topic hz`로 재야 한다(§2.2). |

---

## 0. 결론

| 층 | 권장 | 이유 |
|---|---|---|
| 기체·FC | **Holybro X500 V2 + Pixhawk 6C, PX4 v1.15.x** ((1)과 합의) | PX4 표준 기체다. 기존 SITL 자산(v1.15.4, gz x500)과 같은 계열이다. |
| 제어 인터페이스 | **PX4 offboard 속도 setpoint** `[vx, vy, vz, yaw_rate]` | 행동 공간을 바꾸지 않아도 된다. 시뮬 P 게인(1.8/4.0, `MC_ROLL_P` 6.5)이 PX4 기본값과 같다. DCE가 같은 구조로 zero-shot 실기 비행을 보였다((2)). |
| 미들웨어 | **uXRCE-DDS + ROS 2 Humble**, TELEM2 시리얼 921600 | PX4 공식 RPi 가이드가 있는 경로다. 기존 스택도 같은 구성이다. |
| 컴패니언 | **Raspberry Pi 5**, Ubuntu 22.04 ((1)과 합의) | 정책은 약 0.1 M MAC/step이라 부하가 작다. Jetson Nano(구형)는 Humble을 Docker로만 쓸 수 있어 비권장이다. 트랙 B를 통합할 때는 Orin Nano를 쓴다. |
| 노드 | **Python rclpy 노드 하나**(정책 + 투하 게이트 + 상태기계 + 안전 감시) | 신경망이 작다. 노드를 나누면 DDS 경유만 한 번 더 늘어난다. |
| 정책 이식 | **.npz 가중치 + 순수 numpy 추론**, torch 출력과 1e-5 이내로 일치하는지 테스트 | 탑재 컴퓨터에 torch를 설치할 필요가 없다. |
| 투하 | **`DO_SET_ACTUATOR`(187) → "Peripheral via Actuator Set n" → PWM 서보** | Pixhawk 6C 기본 빌드에서 동작한다. ulog와 같은 시간축에 남는다. |
| 저수준 정합 | **실측 계단응답으로 Isaac 속도 루프를 재정렬하고 시정수 ±10% DR → L0 미세조정 → GRU 재적합** | 저수준 체인 불일치가 실기 오차를 가장 크게 키운다(Learning to Throw ×3.7). |
| sim-to-sim | **Gazebo Harmonic + PX4 SITL**(기존 이미지 재사용, 기체 모델은 실측값으로 갱신). MuJoCo는 쓰지 않는다 | 실기와 바이트 단위로 같은 노드를 실제 PX4 PID 위에서 검증한다. RESC와 Narrow Gaps의 경로와 같다. |

---

## 1. 실기 비행 제어기 선택과 순수 P 대 PID 문제

### 1.1 우리 시뮬 제어기 (코드로 확인)

`isaac_lab/drone_bombard/drone_bombard_env.py`

| 단계 | 위치 | 내용 |
|---|---|---|
| 행동 전처리 | `_pre_physics_step` (1115행) | ±1로 자르고, 레이트 제한 ±0.2/step을 건 뒤 물리 단위(4/3/3/1)로 바꾼다. |
| 저역통과 필터 | `_apply_action` (1147행) | 20 Hz, LPF α=0.4 |
| 속도 루프 | `_run_velocity_controller` (1440행~) | 속도 P `kp_xy 1.8 / kp_z 4.0`, 적분항 없음, 가속 제한 8/4 m/s², 기울기 제한 35°, 추력 상한 40.6 N |
| 자세·각속도 루프 | 같은 함수 | 자세 P 6.5, 각속도 P (18, 18, 8), τ=Iα, 모터 지연 없음 |

노트 `notes/research/legacy/isaac_velocity_controller.md`에 따르면 이 게인은 PX4 `MPC_*_VEL_P_ACC`, `MC_ROLL_P`의 근사다. **PX4 SITL 계단응답과 대조 검정한 적은 없다**(계획한 7개 운용점 중 0개 확보).

### 1.2 PX4 v1.15.4 실제 구조

로컬 이미지 `drone-bombard-px4built:latest`의 `/opt/PX4-Autopilot` 소스(태그 v1.15.4)를 직접 읽었다.

| 항목 | PX4 v1.15.4 기본값 | 우리 시뮬 | 조치 |
|---|---|---|---|
| 속도 루프 | **PID** `acc = P·e + ∫ − D·v̇`. xy는 tracking anti-windup, z 적분은 ±g로 제한 | 순수 P | §1.4 |
| `MPC_XY_VEL_P/I/D_ACC` | 1.8 / 0.4 / 0.2 | 1.8 / – / – | P는 같다 |
| `MPC_Z_VEL_P/I/D_ACC` | 4.0 / 2.0 / 0 | 4.0 / – / – | P는 같다 |
| `MC_ROLL_P` | 6.5 | 6.5 | 같다 |
| 각속도 루프 | K-PID (`MC_ROLLRATE_P/I/D` 0.15/0.2/0.003, 정규화 토크) + 실제 모터 동특성 | 이상적 P, 모터 지연 없음 | 계단응답 동정으로 흡수(§1.4) |
| `MPC_TILTMAX_AIR` | 45° | 35° | **35로 설정** |
| `MPC_Z_VEL_MAX_DN` | **1.5 m/s** (허용 범위 0.5~4) | vz ±3 m/s | **3.0으로 설정**. offboard 속도 모드에서도 `_positionControl`이 `_vel_sp(2)`를 clamp하므로, 그대로 두면 정책의 하강 명령이 잘린다. |
| `MPC_Z_VEL_MAX_UP` / `MPC_XY_VEL_MAX` | 3.0 / 12 | ±3 / 최대 4 | 문제없음 |
| 가속 평활화 | offboard는 `trajectory_setpoint`를 직접 받는다(`MulticopterPositionControl.cpp:391`). FlightTask jerk 평활화를 거치지 않는다 | 8/4 m/s² clamp | 계단응답에서 확인 |
| `MPC_THR_MAX` | 1.0 | 추력 상한 = T/W 2.0 | §1.5 참고 |
| 호버 추력 | 호버 추력 추정기(`MPC_USE_HTE=1`). 질량을 쓰지 않는다 | 질량 믿음 `_ctrl_mass` | 투하 뒤 질량 변화는 PX4가 HTE와 z 적분으로 흡수한다 |
| offboard 하트비트 | 2 Hz 이상, `COM_OF_LOSS_T` 1.0 s | – | 안전 설정(§2.5) |

출처: [PX4 Controller Diagrams](https://docs.px4.io/main/en/flight_stack/controller_diagrams.html), [PX4 Offboard](https://docs.px4.io/main/en/flight_modes/offboard.html). 기본값은 v1.15.4 소스의 `mc_pos_control/*_params.c`에서 읽었다.

### 1.3 PID 차이와 기체 변경이 정책·잔차에 주는 영향

| 영향 | 크기와 근거 |
|---|---|
| **바람 속 정상상태 속도 오차** | 순수 P에서는 상수 바람이 $e_{ss}=a_{wind}/k_p$를 남긴다. PX4는 적분으로 이 오차를 지운다. 대략의 적분 시간은 $k_p/k_i=1.8/0.4\approx4.5$ s(어림 계산)이고 배달 시간은 약 5.8 s다. 따라서 에피소드 중간에 오차가 줄어드는 비정상 구간에 해당한다. |
| **잔차 GRU** | `residual_observability.md` §7: "속도·명령" 누적 4채널(P 제어의 흔적)을 빼도 R²는 0.668에서 0.657로 0.010만 줄어든다. 적합은 자세 평균(힘 균형 $mg\tan\theta=F_{wind}$, 어떤 제어기에서도 존재)에 기댄다. **정보는 남지만 입력 분포가 바뀌므로 재적합해야 한다.** |
| **기체 변경**(2.07 → 약 1.4 kg, T/W 2.0 → 약 3.5, 추정) | $\tan\theta=F_{wind}/(mg)$이므로 질량이 줄면 같은 바람에서 기울기가 약 1.5배가 된다. GRU가 기대는 **자세-바람 관계의 기울기가 바뀐다** → 실기 질량으로 생성한 데이터로 GRU를 재적합해야 한다. T/W가 높아지면 수직 포화 여유가 커지므로 L0에는 대체로 무해할 것으로 본다(추정). |
| **투하 순간 반동** | 투하 직전 추력이 무게와 평형이라고 가정하면 상향 가속 계단은 $g\,(m_{loaded}/m_{empty}-1)$이다. 시뮬은 $9.81\times(2.17/2.07-1)\approx0.47$ m/s², X500 1.4 kg이면 $9.81\times(1.5/1.4-1)\approx0.70$ m/s²(추정). 투하 뒤 거동은 착탄과 무관하지만 POST 상태 안정성에 영향을 준다. |
| **L0 비행 정책** | CCIP 오차로 폐루프를 돌기 때문에 추종이 좋아지는 쪽은 대체로 무해할 것이다(추정, 미검증). 위험한 경우는 세 가지다. (a) 순항 인계 직후 적분 와인드업. (b) 하강 제한 1.5 m/s(미설정 시)로 투하 창에 늦게 도착. (c) 실기 응답이 더 빠르거나 느려서 생기는 레이트 제한·LPF와의 상호작용. |
| **지연** | 시뮬은 관측에서 행동까지의 지연이 0이다. 실기는 DDS, 추론, setpoint 반영, 모터 지연이 더해진다. 측정 전이므로 미확인이다. E2E-Fly는 탑재 실행에서 30 ms 미만, 지상 실행에서 약 90 ms를 보고했다((2)). |

### 1.4 대응 방안

| 방안 | 내용 | 재학습 | 위치 |
|---|---|---|---|
| **A. PX4를 시뮬에 맞춤** | `MPC_XY_VEL_I_ACC=0`, `MPC_XY_VEL_D_ACC` 최소, `MPC_Z_VEL_I_ACC` 최소(0.2), `MPC_TILTMAX_AIR=35`, `MPC_Z_VEL_MAX_DN=3` | 없음 | **SITL·실기 첫 비행용 대비책 겸 ablation 팔** |
| **B. 시뮬을 실측 응답에 맞춤 (본선)** | ① SITL과 실기에서 계단응답 7점 측정(전진 {1,2,4}, 측방 {1.5,3}, 수직 {±1,±3}, 대각). ② Isaac `_run_velocity_controller`에 I, D, anti-windup을 넣거나 1차+지연 모델로 맞춰 축별 시정수와 지연을 재현. ③ 시정수 ±10%, 관측·행동 지연 0~1 step DR. ④ 실측 질량·관성·T/W 반영 | **L0 미세조정**(기존 ckpt에서 출발) + **GRU 재적합**(오프라인) | 논문 본선. 근거는 DCE ±10%, Learning to Throw의 실측 컨트롤러 재현, SimpleFlight의 "정확히 잴 수 있는 값은 SysID로, 민감한 값만 DR"((2)) |

B의 ①은 생략할 수 없다. Learning to Throw ablation에서 저수준 컨트롤러를 이상화했을 때 오차가 3.7배로 커졌고, 제거한 구성요소 가운데 가장 컸다.

### 1.5 T/W 2.0 대 실기 약 3.5(추정)

- **결론: PX4에서 추력을 인위적으로 깎지 않는다**(`MPC_THR_MAX`는 기본값 유지).
- 투하 구간은 기울기 15° 이내라 추력 포화가 거의 일어나지 않는다((1) §2.2). 따라서 행동에 묶이는 제약은 기울기 35°다.
- 그 대신 Isaac 측 `thrust_to_weight_unloaded`를 실측값으로 바꿔 B에 포함한다.
- `MPC_THR_MAX`로 T/W 2.0을 흉내 내는 방법은 정규화 추력 곡선이 비선형이라 정확하지 않다. 바람 속 안전 여유만 줄인다.

### 1.6 다른 펌웨어와 비교

| | PX4 offboard 속도 | ArduPilot GUIDED | Betaflight CTBR |
|---|---|---|---|
| 우리 행동 공간 | 그대로 맞음 | `/ap/cmd_vel`로 가능 ([문서](https://ardupilot.org/dev/docs/ros2-interfaces.html)) | 맞지 않음. 전면 재학습 필요 |
| 게인 대응 | P와 `MC_ROLL_P`가 기본값과 같다 | 대응표 없음 | – |
| 기존 자산 | v1.15.4 SITL, airframe 4015, offboard 노드 | 없음 | 없음 |
| 선행 | Neural-Fly(PX4 + RPi4, 2.6 kg), RESC·Narrow Gaps(PX4), Ishihara(PX4 PID + 잔차) | DCE(ArduPilot 속도 + 요레이트 10 Hz) | Learning to Throw(0.21 kg, T/W 6.8), Swift(870 g, T/W 4.1) |
| 판정 | **채택** | 대안이지만 이점이 없다 | 2 kg급·T/W 2~3.5에 부적합 |

PX4 안에서 정책을 직접 돌리는 `mc_nn_control`(v1.17+)과 RAPTOR(v1.18)는 모터 수준 end-to-end 정책용이라 해당하지 않는다 ([docs](https://docs.px4.io/main/en/neural_networks/mc_neural_network_control.html), [RAPTOR](https://docs.px4.io/main/en/neural_networks/raptor)). NN-PX4(FC 위 93 µs, (2))는 정책이 작으면 FC에 올릴 수도 있다는 근거일 뿐이다. 우리 구조에서는 컴패니언이 더 단순하다.

---

## 2. 배포 소프트웨어 구조

### 2.1 노드 구성 (단일 노드)

```
Pixhawk 6C (PX4 v1.15)
  uxrce_dds_client ──TELEM2 921600──> MicroXRCEAgent (RPi 5)
                                          │
  /fmu/out/vehicle_odometry ─┐            │
  /fmu/out/vehicle_status ───┤            │
                             ▼            │
  bombard_node (rclpy) ──────────────────┘
   ├ state_cb : 최신 odometry 저장 (NED/FRD → 과제 프레임 ENU/FLU)
   ├ 10 Hz    : obs26 조립 → L0 MLP → [v4, drop, (res 무시)]
   │            GRU(obs26, h) → EMA α=0.3 → residual(m)
   │            rate-limit ±0.2 → 물리 스케일
   ├ 20 Hz    : LPF α=0.4 → TrajectorySetpoint(vel NED, yaw=NaN, yawspeed) + OffboardControlMode
   ├ 100 Hz   : 투하 게이트(release_gate + crossing, wants_drop 유지) → VehicleCommand DO_SET_ACTUATOR
   └ 상태기계 : IDLE→TAKEOFF→APPROACH(스크립트 순항)→HANDOFF(t=0)→POLICY→POST(제동·호버)→LAND
  rosbag2 + PX4 ulog (둘 다 기록)
```

**시뮬과 맞춰야 하는 순서와 상태**

1. tick t의 관측에는 **직전 tick의 잔차(EMA 상태)**가 들어간다. `task_env._ccip`가 `_residual_action`을 쓰기 때문이다.
2. 시뮬은 `_resolve_release`를 100 Hz(`decide_at_physics_rate`)로 돌린다. `wants_drop`는 정책 주기 동안 유지하고, 발사 순간은 `d_impact`가 더 이상 줄지 않는 서브스텝으로 정한다. `math_utils.release_gate`를 numpy로 옮겨 그대로 재현한다.
3. **EMA 이중 적용 함정**: `play.py`의 `_SLResidual`에는 자체 EMA(`--sl_ema`)가 있고, env에는 `residual.ema_alpha`가 따로 있다. 본 결과(CEP50 0.204)를 낸 평가가 어느 쪽을 적용했는지 확인한 뒤 **한쪽만** 이식한다.
4. **순항 인계**: 시뮬 reset은 순항 속도에서 `_prev_action`과 `_v_filt`를 시드한다(`task_env.py` 975~987행). 실기 HANDOFF에서도 같은 값으로 시드한다. 방안 B를 적용했다면 PX4 적분 상태는 순항 중에 이미 수렴해 있으므로 와인드업 문제는 작다(추정).

### 2.2 지연 예산 (10 Hz = 100 ms)

| 구간 | 예상 | 측정 방법 |
|---|---|---|
| PX4 EKF → DDS → 노드 | 수 ms ~ 수십 ms (미확인) | `timestamp_sample`과 노드 수신 시각의 차이 |
| `vehicle_odometry` 수신률 | **미확인**. v1.15.4 `dds_topics.yaml`에 `rate_limit`이 지정되어 있지 않다 | `ros2 topic hz`. 100 Hz 게이트는 이 값이 50 Hz 이상일 때 의미가 있다. 낮으면 게이트에서 odometry를 외삽한다. |
| 추론 (MLP 약 7.5만 파라미터 + GRU 64) | 1 ms 미만 (추정) | RPi 5에서 `time.perf_counter` |
| setpoint → PX4 반영 | 수 ms (미확인) | ulog `trajectory_setpoint` 타임스탬프 |
| 서보 기구 지연 | 시뮬 가정 0.22 ± 0.05 s | 벤치 실측 (§5) |

측정한 관측→행동 지연은 방안 B의 지연 DR 범위로 쓴다.

### 2.3 정책 export

| 방식 | 판정 |
|---|---|
| **순수 numpy** (.npz: L0 actor 2×256 ELU, GRUCell 가중치, `mu`) | **채택**. `empirical_normalization=False`라 정규화 통계가 필요 없다. |
| TorchScript | GRU는 이미 이 형식이다(`_fit_sl_seq.py`). ARM에 torch를 설치하는 비용이 든다. |
| ONNX Runtime | 이득이 없다. |

**필수 검사**: Isaac 평가 롤아웃에서 (obs, L0 출력, GRU 출력)을 덤프하고, numpy 구현이 1e-5 이내로 재현하는지 `assert`로 확인한다.

### 2.4 좌표계

PX4 `vehicle_odometry`(v1.15.4 msg): position·velocity는 NED, `q`는 FRD body → NED, `angular_velocity`는 FRD body다.

| 양 | 변환 (NED/FRD → 과제 프레임 ENU/FLU) |
|---|---|
| 위치·속도 | $(x_E, x_N, -x_D)$로 바꾼 뒤 과제 프레임 회전 $R(-\psi_{task})$ 적용 |
| 자세 | $q_{ENU\leftarrow FLU}=q_{ENU\leftarrow NED}\otimes q\otimes q_{FRD\leftarrow FLU}$ 후 Isaac의 `euler_xyz_from_quat`를 numpy로 포팅해 사용 |
| 각속도 | $(p, -q, -r)$ |
| 명령 | $v_{NED}=(v_N, v_E, -v_{up})$, `yawspeed` = −yaw_rate, `yaw` = NaN, `position` = NaN |

- **과제 프레임**: 원점은 표적(RTK로 측량한 좌표), +x는 접근 방위다. 시뮬은 순항 방위가 ±30°이고 vx/vy 스케일이 4/3로 비대칭이므로 접근 축을 반드시 +x로 둔다.
- **교훈(memory `coordinate-frames`)**: 좌표 진단은 ground truth(Gazebo `gz model -p`, RTK 또는 모캡)와 동시에 측정한다. 예전 노드의 `cmd.y = +West` 규약은 재사용하지 않는다.

### 2.5 안전장치

| 장치 | 구현 |
|---|---|
| 이륙 전 Go/No-go | 지상 풍속계의 1분 평균풍이 운용 상한(초기값 5 m/s, (1)의 권고이며 근거 실험 없음)을 넘으면 비행하지 않는다. 정책 입력에는 쓰지 않는다. |
| 지오펜스 | PX4 `GF_*` + 노드 내부 회랑 다각형 |
| offboard 손실 | `COM_OF_LOSS_T` 0.5 s, `COM_OBL_RC_ACT` = Hold 또는 Land. 값과 의미의 대응은 문서로 재확인해야 한다. |
| 수동 전환 | RC 모드 스위치(Position)를 최우선으로 둔다. 킬 스위치 |
| 정책 감시 | 기울기 30° 초과, 고도 2.5 m 미만, 회랑 이탈, HANDOFF 후 20 s 초과 중 하나라도 걸리면 Hold |
| 투하 인터록 | RC 투하 허가 ∧ 투하 구역 다각형 안 ∧ `payload_attached` ∧ 게이트 ∧ 에피소드당 1회. 서보 failsafe·disarmed 값은 "닫힘" |
| uXRCE 보안 | DDS에 인증이 없으므로 직결 케이블만 쓴다 ([문서](https://docs.px4.io/main/en/middleware/uxrce_dds.html)) |

---

## 3. 관측 26채널: 실기 출처와 정의 일치 검증

`task_env._get_observations`(832행)에서 확인했다. 0~4번은 표적 탐지(`d_xy ≤ 7 m`) 전에는 0이다.

| # | 정의 (시뮬) | 정규화 | 실기 출처 |
|---|---|---|---|
| 0–1 | 표적 − 기체 xy | /20, clip | 측량한 표적 좌표 − RTK·EKF 위치(과제 프레임) |
| 2–3 | CCIP 오차 = (공칭 착탄 + 잔차) − 표적 | /10 | `_nominal_impact` numpy 포팅 + GRU/EMA |
| 4 | ‖CCIP 오차‖ | /10, [0,1] | 같음 |
| 5 | 0 − 고도 | /20 | **표적 지면 기준 높이**. RTK 높이에서 측량한 표적 높이를 뺀다 |
| 6–8 | 월드 속도 | /10 | odometry velocity |
| 9–10 | roll, pitch | /π | q 변환 |
| 11–12 | sin, cos yaw | – | q 변환 + 과제 프레임 |
| 13–15 | 바디 각속도 | /π | $(p, -q, -r)$ |
| 16 | 낙하 시간 $t_f$ | /5 | 계산 |
| 17 | 수평 속력 | /10 | 계산 |
| 18 | 고도 | /20, [0,1] | 5번과 같은 기준 |
| 19 | payload_attached | – | 노드 상태(발사 + 지연 뒤 0) |
| 20 | 1 − t/20 s | – | HANDOFF부터 노드 시계 |
| 21–24 | 직전 레이트 제한 후 정규화 명령 | – | 노드 내부 상태 |
| 25 | 탐지 플래그 | – | 참값 거리 ≤ 7 m (트랙 A) |

**주의할 점**

- 2~4번은 페이로드 탄도계수 k/m, `payload_mount_z`, 릴리즈 지연 평균에 의존한다. 이 값들은 **실측해서 넣는다**(낙하 시험, CAD, 벤치 측정).
- 고도는 RTK 높이를 EKF2 높이 기준으로 쓴다((1)의 F9P). baro만 쓰면 드리프트가 낙하시간 오차로 바로 이어진다. 실내에서는 모캡을 외부 비전으로 융합한다.
- 시뮬 관측 노이즈(`_perturb_obs`)는 배포 때 넣지 않는다.

**일치 검증 4단계**

1. **골든 테스트**: Isaac 원시 상태를 NED/FRD로 역변환해 numpy 조립기에 넣고, 같은 obs26이 1e-5 이내로 나오는지 확인한다.
2. **SITL 대조**: Gazebo ground truth와 PX4 odometry로 조립한 obs를 동시에 기록해 비교한다.
3. **지상 부호 시험**: 기체를 손으로 기울이거나 옮기며 6, 7, 9, 10번 채널의 부호를 확인한다.
4. **섀도 비행**: 조종사가 Position 모드로 비행하는 동안 노드는 obs와 행동을 계산만 하고 보내지 않는다.

---

## 4. sim-to-sim 단계

### 4.1 기존 스택 재사용

| 자산 | 상태 | 조치 |
|---|---|---|
| Docker `drone-bombard-px4built:latest` | PX4 **v1.15.4**(태그 확인), ROS 2 Humble, gz-sim 8.11 | 그대로 쓴다 |
| airframe `4015_gz_x500_bombard` | `MPC_THR_HOVER 0.60`, `COM_ARM_WO_GPS 1` | §1.2 파라미터(`TILTMAX 35`, `Z_VEL_MAX_DN 3`)를 추가한다 |
| `gazebo_models/x500_bombard/model.sdf` | 본체 2.0 kg, ixx 0.02167, DetachableJoint | **실측 X500 V2 질량·관성·모터 상수로 갱신** |
| `payload_cylinder` | 0.1 kg | 실측 페이로드로 맞춘다 |
| `worlds/x_marker_world.sdf` | 바람 없음 | `WindEffects` 추가 ([API](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1WindEffects.html)) |
| `ros2_ws/src/drone_controller` | offboard 하트비트, TrajectorySetpoint, PX4 QoS | 새 노드의 뼈대로 쓴다 |
| `ros2_ws/src/px4_msgs` | 작업 트리에서 비어 있음 | v1.15.4에 맞는 버전을 받는다 |

추가 작업:

- 낙하 중인 페이로드의 이차 항력이 Gazebo 기본 기능으로 되는지는 **미확인**이다. 안 되면 노드에서 착탄을 해석적으로 계산하거나 작은 플러그인을 만든다.
- 측정한 서보 지연만큼 detach 토픽을 늦춘다.

### 4.2 이 단계가 검증하는 것

- 실제 PX4 PID, 하강 제한, offboard 인계와 와인드업
- DDS 타이밍, 좌표 변환, 투하 명령 체인, 안전 로직
- **실기에 올릴 노드와 바이트 단위로 같은 노드**
- 방안 B의 계단응답 7점(SITL 판)을 1차로 측정

물리 충실도 검증은 아니다. 선행연구(RESC, Narrow Gaps)도 이 단계를 인터페이스 검증 용도로 썼고, Narrow Gaps는 일부러 다른 동역학을 넣었다((2)). 통과 기준 제안은 다음과 같다. Isaac과 같은 조건(무풍, 상수풍 2종)에서 CEP50이 1.5배 이내이고, 좌표·부호 오류와 하강 포화가 0건일 것. 최종 기준은 (5)에서 정한다.

### 4.3 대안 비교

| | Gazebo+PX4 SITL | MuJoCo | Pegasus (Isaac Sim + PX4) |
|---|---|---|---|
| 실제 PX4 제어기 | 그대로 | 다시 구현해야 함 | 그대로 |
| 기존 자산 | 있음 | 없음 | 없음 |
| DDS 경로 | 검증됨 | – | MAVLink 위주, DDS는 미확인 |
| 판정 | **채택** | 쓰지 않음 | 선택 사항(같은 물리에서 PX4만 바꾸는 ablation용) |

---

## 5. 투하 메커니즘 펌웨어와 지연 측정

| 방법 | v1.15.4 확인 결과 | 판정 |
|---|---|---|
| **`VEHICLE_CMD_DO_SET_ACTUATOR`(187)** → "Peripheral via Actuator Set 1..6", 값 −1~1 | 기본 빌드에서 동작 ([docs](https://docs.px4.io/main/en/payloads/generic_actuator_control.html)) | **채택** |
| `DO_GRIPPER`(211) + `payload_deliverer`(`PD_GRIPPER_*`) | `CONFIG_MODULES_PAYLOAD_DELIVERER`는 **sitl 보드 설정에만** 있고, fmu-v6c/v6x에는 없다(로컬 소스에서 재확인). 커스텀 빌드가 필요하다 ([docs](https://docs.px4.io/main/en/peripherals/gripper.html)) | 대안. (1)의 권장안을 이 경로로 대체한다 |
| 컴패니언 GPIO로 PWM 직접 출력 | ulog 시간축 밖에 있다 | 쓰지 않음 |
| `/fmu/in/actuator_servos` | offboard actuator 모드가 필요하다 | 쓰지 않음 |

PX4는 투하 기구 하드웨어 상태를 알지 못한다. 따라서 지연은 직접 재야 한다.

**지연 측정**

1. **벤치 시험, 30회 이상((1)과 통일)**: 같은 명령으로 Actuator Set 2에 LED를 켠다. 240 fps 이상으로 촬영해 LED 점등 프레임과 분리 프레임의 차이를 잰다. ulog `vehicle_command` 시각과 노드 송신 시각으로 전송 지연과 기구 지연을 나눈다.
2. **비행 중 교차 검증**: 투하 순간 상향 가속 계단(X500 1.4 kg이면 약 0.7 m/s², 추정)을 ulog `sensor_combined`에서 검출한다.
3. 측정한 평균과 표준편차를 `release_delay_mean/std`(현재 0.22/0.05)와 공칭 예측기에 반영한다. 평균은 예측기가 보상하므로, 남는 오차원은 **표준편차**다((1)).

---

## 6. 재학습이 필요한 변경과 필요 없는 변경

| 분류 | 항목 |
|---|---|
| **재학습 없음** (배포 측 작업) | 좌표 변환과 과제 프레임 / LPF·레이트 제한·EMA를 노드에서 재현 / numpy export / PX4 파라미터(`TILTMAX 35`, `Z_VEL_MAX_DN 3`) / 안전장치와 인터록 / 서보 명령 / 순항 인계 시드 / 방안 A로 첫 SITL·실기 비행 |
| **예측기 파라미터 교체 + GRU 오프라인 재적합** | 실측 릴리즈 지연 / 페이로드 k/m / `payload_mount_z` / 고도 기준 |
| **L0 미세조정 + GRU 재적합** (**기본 경로**) | 실측 X500 질량·관성·T/W 반영 / 계단응답에 맞춘 속도 루프(I, D 또는 1차+지연) + 시정수 ±10% DR / 관측·행동 지연 DR / 시나리오 스케일을 바꿀 경우((3)의 결정) |
| **전면 재학습** | 행동 공간 변경(CTBR). 권장하지 않음 |

- GRU는 어느 경우든 **실기와 같은 제어기·기체로 생성한 시뮬 데이터로 재적합**한다(`residual_observability.md` §7). PPO 없이 수 분 수준이다.
- L0 미세조정은 기존 ckpt에서 출발한다. 비용은 미측정이다.
- 순서: 계단응답 측정(SITL → 실기 호버·저속) → Isaac 재정렬 → L0 미세조정 → GRU 재적합 → SITL 회귀 → 실기 투하.
- 실기 결과와 시뮬 사이 차이가 크면, Swift처럼 실비행 로그(RTK 참값)로 잔차 동역학을 만들어 다시 학습하는 단계를 둔다((2)).

---

## 관련 파일

- /opt/drone-bombard/Drone-Bombard-Simulation/isaac_lab/drone_bombard/drone_bombard_env.py (1115, 1147, 1193, 1440행~)
- /opt/drone-bombard/Drone-Bombard-Simulation/isaac_lab/drone_bombard/task_env.py (740, 794, 832, 899, 975~987행)
- /opt/drone-bombard/Drone-Bombard-Simulation/isaac_lab/drone_bombard/math_utils.py (`release_gate`, `tilt_channels`)
- /opt/drone-bombard/Drone-Bombard-Simulation/isaac_lab/play.py (431행 `_SLResidual`, 578행 잔차 주입)
- /opt/drone-bombard/Drone-Bombard-Simulation/notes/research/legacy/isaac_velocity_controller.md, notes/research/sim2real_gap.md, notes/research/residual_observability.md
- /opt/drone-bombard/Drone-Bombard-Simulation/ros2_ws/src/drone_controller/drone_controller/drone_controller_node.py
- /opt/drone-bombard/Drone-Bombard-Simulation/gazebo_models/x500_bombard/model.sdf, gazebo_models/worlds/x_marker_world.sdf
- PX4 v1.15.4 소스: 이미지 `drone-bombard-px4built:latest:/opt/PX4-Autopilot` (`boards/px4/{sitl,fmu-v6c,fmu-v6x}/*.px4board`, `src/modules/uxrce_dds_client/dds_topics.yaml`, `src/modules/mc_pos_control/*params*.c`)

## 출처

- PX4 문서: [Offboard](https://docs.px4.io/main/en/flight_modes/offboard.html) · [Controller Diagrams](https://docs.px4.io/main/en/flight_stack/controller_diagrams.html) · [uXRCE-DDS](https://docs.px4.io/main/en/middleware/uxrce_dds.html) · [RPi companion](https://docs.px4.io/main/en/companion_computer/pixhawk_rpi.html) · [Gripper](https://docs.px4.io/main/en/peripherals/gripper.html) · [Generic Actuator Control](https://docs.px4.io/main/en/payloads/generic_actuator_control.html) · [mc_nn_control](https://docs.px4.io/main/en/neural_networks/mc_neural_network_control.html) · [RAPTOR](https://docs.px4.io/main/en/neural_networks/raptor) · [Simulation](https://docs.px4.io/main/en/simulation/)
- 기타 도구·문서: [ArduPilot ROS 2](https://ardupilot.org/dev/docs/ros2-interfaces.html) · [Pegasus](https://pegasussimulator.github.io/PegasusSimulator/) · [gz-sim WindEffects](https://gazebosim.org/api/sim/8/classgz_1_1sim_1_1systems_1_1WindEffects.html) · [Jetson Nano Humble (NVIDIA forum)](https://forums.developer.nvidia.com/t/ros2-humble-desktop-for-jetson-nano/236942)
- 선행연구((2)에서 확인한 내용을 인용): [Learning to Throw, arXiv 2606.27603](https://arxiv.org/html/2606.27603) · [DCE, ICRA 2024](https://ar5iv.labs.arxiv.org/html/2402.03947) · [SimpleFlight, RA-L 2025](https://arxiv.org/html/2412.11764v4) · [Neural-Fly](https://arxiv.org/abs/2205.06908) · [RESC](https://arxiv.org/html/2408.00275) · [Narrow Gaps](https://ar5iv.labs.arxiv.org/html/2302.11233) · [Ishihara residual RL](https://ar5iv.labs.arxiv.org/html/2308.01648) · [E2E-Fly](https://arxiv.org/html/2604.12916) · [NN mode for PX4](https://arxiv.org/html/2505.00432) · [Swift](https://pmc.ncbi.nlm.nih.gov/articles/10468397)
- 하드웨어((1)에서 인용): [Holybro X500 V2](https://docs.holybro.com/drone-development-kit/px4-development-kit-x500v2) · [KOTSA 조종자 증명](https://main.kotsa.or.kr/portal/contents.do?menuCode=02020200)

**미확인 항목**

- `vehicle_odometry`의 실제 DDS 수신률
- DDS·추론·setpoint 지연 실측값
- Gazebo에서 페이로드 이차 항력 구현 가능 여부
- `COM_OBL_RC_ACT` 값과 동작의 정확한 대응
- X500 V2 실측 질량·관성·추력 곡선
- 본 결과 평가에 쓰인 EMA 적용 위치(play 쪽인지 env 쪽인지)