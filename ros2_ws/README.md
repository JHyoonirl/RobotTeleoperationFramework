# Neuromeka Indy7 + ROS 2 + Crocoddyl

공식 `neuromeka-robotics/indy-ros2`에서 Indy7에 필요한 것만 가져왔다.

- 공용 로봇 모델 package: `src/indy7_description` (URDF와 visual/collision STL)
- Indy7 링크의 visual/collision STL 14개
- ROS 2 모델, MPC 계산, 실기 통신을 독립 package로 분리
- `indy_driver`, `indy_gazebo`, `indy_moveit`, `indy_interfaces`는 복사하지 않음
- `indy7_dcp3`: Neuromeka Python DCP3 client를 쓰는 제한된 실기 adapter

Indy7은 제품명이 7이지만 실제 관절은 6개다. 기본 MPC 상태는
`x=[q, qdot]`, 내부 최적화 입력은 URDF effort limit 안의 관절 토크 `u=tau`다.
Pinocchio ABA가 질량행렬·중력·코리올리 항을 반영해 다음 상태를 예측한다.
매 주기 FDDP가 목적함수를 풀고 첫 번째 예측 상태의 `q_cmd`를
라디안 절대 관절 위치로 출력한다. 시뮬레이션에서는 이 토픽을
`forward_position_controller`에 연결하고, 실기에서는 같은 토픽을
IndyDCP3 `TELE_JOINT_ABSOLUTE / movetelej_abs()` adapter에 연결한다.

## Docker 실행환경

이 문서의 모든 ROS 2 명령은 Docker 컨테이너 안에서 실행한다. 호스트에서 ROS 2를
설치하거나 `.venv`를 만들거나 `rosdep`를 실행하지 않는다. MPC image는
[`requirements-mpc.txt`](../robots/indy7/requirements-mpc.txt)의 NumPy 2 / Pinocchio /
Crocoddyl Python stack을 사용한다. IndyDCP3 image는
[`requirements-hardware.txt`](../robots/indy7/requirements-hardware.txt)의 Neuromeka SDK와
NumPy 1 계열을 사용한다. Python ABI 충돌 방지를 위해 두 실행환경을 별도 이미지로 유지한다.
모든 프로젝트 컨테이너와 `make run-mpc`는 `ROS_DOMAIN_ID=77`을 사용한다. 호스트에서 ROS 2
명령을 직접 실행하는 경우에도 `export ROS_DOMAIN_ID=77`을 설정해 같은 DDS domain에 연결한다.

IndyDCP3 통신 image에는 mock simulation과 시각화 의존성을 넣지 않는다. MPC image의 ROS
패키지는 mock 시뮬레이션용 `ros2_control`, `controller_manager`,
`joint_state_broadcaster`, `position_controllers`, `robot_state_publisher`를 설치한다.
`rviz2`와 `joint_state_publisher_gui`는 선택형 `mpc-gui` 이미지에만 포함된다.
이 구성은 공식 `indy_driver` ROS 패키지를 실행하지 않으므로
MoveIt, Gazebo, RealSense, `joint_trajectory_controller`는 설치하지 않는다. 공식
드라이버 전체를 별도로 사용할 때만 해당 README의 나머지 의존성이 필요하다.

저장소 루트에서 이미지를 빌드하고 테스트한다.

```bash
cd /path/to/RobotTeleoperationFramework
make docker-build
make docker-test
```

두 이미지의 태그는 `robot-teleoperation-framework:mpc`와
`robot-teleoperation-framework:dcp3`다. `make docker-run`은 MPC 컨테이너 셸을 연다.

이미지 안에는 다음 경로로 빌드된 ROS 2 workspace가 있다.

```text
/opt/robot_teleoperation_framework/src
/opt/robot_teleoperation_framework/install
```

이미지는 계산(MPC)과 제조사 통신(DCP3) 역할에 따라 분리되어 있다.

## 실행

일반 개발 흐름은 저장소 소스와 overlay를 연결하는 `make docker-run-gui`입니다. GUI image가
없을 때만 준비한 뒤 셸을 열며, ROS node/launch는 자동 실행하지 않습니다. 셸 안에서 아래처럼
원하는 launch를 직접 실행합니다. 반대로 이어지는 `docker run ...:mpc` 예제는 저수준 standalone
실행법으로, image의 기본 CMD가 headless MPC launch를 자동 실행합니다. 소스 마운트 기반 개발
중에는 기본 CMD를 쓰지 말고 `make docker-run-gui`를 사용하세요.

```bash
make docker-run-gui
# 컨테이너 셸 안에서
ros2 launch indy7_mpc indy7_mpc.launch.py simulate:=true rviz:=true plot:=true \
  diagnostic_log_directory:=/tmp/indy7_sim_logs
```

```bash
docker run --rm --net=host --ipc=host \
  robot-teleoperation-framework:mpc
```

위 명령은 이미지에 정의된 headless 기본 launch를 실행한다. launch 인자를 바꾸려면
이미지의 기본 명령을 덮어쓴다.

```bash
docker run --rm --net=host --ipc=host \
  robot-teleoperation-framework:mpc \
  ros2 launch indy7_mpc indy7_mpc.launch.py rviz:=false plot:=false
```

토픽 확인이나 로그 분석을 위해 컨테이너를 백그라운드에서 유지하려면 이름을 지정한다.

```bash
docker run -d --name robot-teleoperation --net=host --ipc=host \
  robot-teleoperation-framework:mpc
docker logs -f robot-teleoperation
```

이후의 `docker exec robot-teleoperation ...` 명령은 이 컨테이너가 실행 중이라는 전제다.

기본값은 `config/indy7_mpc.yaml`을 읽는 시뮬레이션 모드이며, `tcp` frame의 SE(3) 목표를 직접 추적한다. DLS IK는 `qref` 진단용으로만 남아 있으며,
MPC의 tracking residual은 Pinocchio forward kinematics로 매 stage 계산된다.
`ros2_control_node`가 `mock_components/GenericSystem`과 함께 시작되고,
`joint_state_broadcaster` 및 `forward_position_controller`가 활성화된다.
MPC는 매 주기 `mpc/joint_position_cmd`에 `Float64MultiArray([q0..q5])`를
발행하며 launch에서 `/forward_position_controller/commands`로 remap한다.
MPC `dt: 0.005`, ros2_control `update_rate: 200`, DCP3 `publish_rate: 200.0`으로
제어·피드백 주기 설정을 모두 200 Hz로 맞춘다. Mock 하드웨어의 `calculate_dynamics`를
켜서 위치 명령의 실제 반영 차분으로 관절 속도를 계산하고 `/joint_states`에 발행한다.
실기에서는 DCP3가 제공하는 실제 `qdot`을 사용한다.
200 Hz는 설정 목표 주기이며 Python MPC 계산이 5 ms를 넘으면 실제 최적화 빈도는
낮아진다. 이동 중 `solver.solve_time_ms`와 연속 `elapsed_s` 차이로 실제 주기를
확인한다. 실기 DCP3 루프의 실제 주기는 제어기 RPC 지연에도 영향을 받는다.
MPC의 관절 상태 구독과 관절 명령 발행은 깊이 1로 최신 값만 유지한다.
계산이 5 ms를 넘을 때 과거 상태가 큐에 누적되어 지연된 보정을 반복하지 않도록 한다.
Mock 위치 서보의 속도는 MPC가 실제 수신한 위치·타임스탬프 간격으로 재계산하고,
시정수 `2*dt`(현재 10 ms)로 평활화한다. 실기의 실제 속도 피드백에는 이 처리를 적용하지 않는다.
시뮬레이션에서 목표 위치·방향이 `stationary_hold_position_tolerance`와
`stationary_hold_orientation_tolerance`(현재 2 mm, 0.01 rad) 안에 들어오면 해당 관절
위치를 유지한다. 새 목표 입력이 오면 최적화를 재개한다. 실기는 기존의 솔버 정지 판정을 유지한다.
Task-space 목표는 horizon의 모든 단계에서 같은 최종 pose를 추적하지 않고,
현재 TCP에서 최종 목표까지의 직선 위치 기준과 SO(3) 방향 보간 기준으로
분해된다. 코드 parameter 기본값은 `cartesian_max_speed=0.05 m/s`,
`cartesian_max_angular_speed=0.75 rad/s`, `cartesian_speed_margin=0.5`지만,
실행 시 읽는 [`indy7_mpc.yaml`](src/indy7_mpc/config/indy7_mpc.yaml)이 이를
덮어쓴다. 현재 YAML은 각각 `0.5 m/s`, `3.0 rad/s`, `1.0`으로 설정되어 있다.
실제 Cartesian 선속도 기준은 이 상한과 현재 방향의 Jacobian/URDF joint-velocity
제한으로 계산한 값 중 작은 값이다. `cartesian_reference_topic`에서 현재 horizon 첫 번째 기준 pose를
확인할 수 있다.
RViz 설정(`config/indy7.rviz`)은 로봇 Alpha를 `0.5`로 표시하고 `tcp` TF 기준의 TCP Axes display를 항상 켠다.
launch를 실행하면 `mpc_plotter`가 함께 열려 position/orientation 오차, 전체 cost, solver 성공 여부·계산시간, control norm, 최소 joint-limit 여유를 실시간으로 표시한다. 주기적 plot 갱신은 창 포커스를 반복해서 가져오지 않는다. GUI가 없는 환경에서는 `plot:=false`로 끌 수 있다.

동시에 진단 로그가 기본적으로 컨테이너 내부의
`/opt/robot_teleoperation_framework/logs/indy7_mpc/<UTC timestamp>/trace.jsonl`에 저장된다.
각 줄은 한 MPC 주기의 JSON record이며 현재/목표 pose, local SE(3) 오차, q/qdot/qref,
hard·soft joint-limit 여유, raw torque와 ABA 동역학 항, cost 항, solver 결과, TCP Jacobian의
singular value/condition number, 월드 Z 회전에 필요한 추정 qdot, teleop pose 갱신 주기와
목표 각속도, joint-state age, 전체 예측 `xs/us`를 포함한다.
같은 디렉터리의 `parameters.json`은 해당 실행의 파라미터 snapshot이다.
`state_input.age_s`는 콜백 수신 이후 시간이며, `state_input.message_age_s`는
ROS 타임스탬프 기준 메시지 생성 이후 시간이므로 큐 지연까지 포함한다.
기록을 끄거나 저장 위치를 바꾸려면 YAML의 `diagnostic_logging`,
`diagnostic_log_directory`, `diagnostic_log_prediction`을 수정한다.
`--rm`으로 컨테이너를 제거하면 컨테이너 내부 로그도 함께 사라지므로 보존하려면
앞의 백그라운드 실행 명령 대신 호스트 디렉터리를 로그 경로에 bind mount한다.

```bash
mkdir -p .docker-data/logs
docker run -d --name robot-teleoperation --net=host --ipc=host \
  -v "$PWD/.docker-data/logs:/opt/robot_teleoperation_framework/logs" \
  robot-teleoperation-framework:mpc
```

토픽은 다음과 같다.

- 입력: `teleop/target_pose` (`geometry_msgs/PoseStamped`)
- 관절 명령: `mpc/joint_position_cmd` (`std_msgs/Float64MultiArray`, rad 절대 q_cmd)
- 예측 출력: `mpc/command_pose` (`geometry_msgs/PoseStamped`, q_cmd의 TCP)
- Cartesian 기준: `mpc/cartesian_reference_pose` (`geometry_msgs/PoseStamped`)
- 목표/현재: `mpc/desired_pose`, `mpc/current_pose` (`geometry_msgs/PoseStamped`)
- 상태: `mpc/cost` (`std_msgs/Float64`, 전체 horizon cost)
- cost term: `mpc/cost/se3_tracking`, `mpc/cost/posture`,
  `mpc/cost/velocity`, `mpc/cost/torque`, `mpc/cost/joint_limit`,
  `mpc/terminal_cost`
- 작업 오차: `mpc/task_error`, `mpc/position_error`, `mpc/orientation_error`
- 진단: `mpc/qref`, `mpc/solve_time_ms`, `mpc/solver_iterations`,
  `mpc/solver_success`, `mpc/control_norm`

실시간 plot 창은 각 cost term을 별도 선으로 표시하며 기본적으로 최근 4초만
표시한다(`history_seconds` 파라미터로 변경 가능). 저장된 로그를 다시
그리려면 다음을 실행한다.

```bash
docker exec robot-teleoperation bash -lc \
  'source /opt/ros/jazzy/setup.bash && \
   source /opt/robot_teleoperation_framework/install/setup.bash && \
   ros2 run indy7_mpc plot_mpc_log \
   /opt/robot_teleoperation_framework/logs/indy7_mpc/<UTC timestamp>'
```

실행 디렉터리의 `plots/overview.png`, `plots/cartesian_path.png`,
`plots/summary.json`이 생성된다. `summary.json`에는 cost·tracking·solver·ABA
동역학의 최종값/평균/최댓값이 들어 있다.

입력 pose의 `header.frame_id`는 기본적으로 `world`여야 한다. 다른 frame을 쓰려면 먼저 TF 변환을 추가하거나 `pose_frame`을 맞춘다. `q_cmd`에는 URDF hard joint limit, measured-q 기준 velocity limit, cycle increment limit을 모두 적용한다. 실기에서는 `mpc/joint_position_cmd`를 IndyDCP3 adapter의 입력으로 사용하고, adapter에서 rad→deg 변환과 deadman/명령 watchdog을 적용한다. 물리 E-stop은 별도로 준비해야 한다. 실행 중 Crocoddyl 전체 목적함수 값은 다음 토픽에서 확인할 수 있다.

현재 적용 중인 Cartesian 속도 관련 YAML 파라미터는 다음과 같다.

```yaml
cartesian_max_speed: 0.5          # m/s, 직선 이동 기준 상한
cartesian_max_angular_speed: 3.0  # rad/s, orientation 상한
cartesian_speed_margin: 1.0       # Jacobian 제한 계산에 적용할 URDF 속도 비율
cartesian_min_duration: 0.10    # 너무 짧은 목표 이동 방지 [s]
```

`cartesian_max_speed`는 계획 기준의 상한이지 측정 TCP가 그 속도로 움직인다는 보장은
아니다. 실제 선속도는 Jacobian 제한, MPC의 관절 명령 필터, DCP3의 속도·가속도
비율에도 영향을 받는다. DCP3 adapter의 `max_joint_velocity`와 주기별 목표값
커팅은 제거했다. [`indy7_dcp3.yaml`](src/indy7_dcp3/config/indy7_dcp3.yaml)의
`velocity_ratio`와 `acceleration_ratio`는 각각 20%이며 TCP 선속도 단위가 아니다.

끝단 선속도를 더 높이는 실험은 먼저 시뮬레이션에서 확인하고, 실기에서는 작업공간을
비운 뒤 물리 E-stop에 손이 닿는 상태에서 작은 직선 이동으로 한 설정만 단계적으로
변경한다. 호스트에서 위 YAML의 `cartesian_max_speed`를 조정하고 MPC launch를 재시작하면
된다. 소스가 `/workspace`에 마운트되어 있으므로 일반 YAML 수정은 Docker 이미지 빌드나
colcon 재빌드가 필요 없다. `cartesian_path.speed_limit_m_s`는 Jacobian 제한까지 반영한
계획 속도 상한이며 측정 속도 자체는 아니다. 실제 응답은 연속된 `/mpc/current_pose` 또는
진단 로그의 현재 pose/time을 비교해 확인한다. 속도 실험 중에는 orientation 상한이나
DCP3 속도·가속도 비율을 동시에 변경하지 않는다.

특이점 근처에서는 계획 속도가 자동으로 낮아질 수 있다. teleoperation pose가 연속해서
들어오는 동안에는 path 시작점을 매번 재설정하지 않고, 현재 line의 목표점만
갱신한다. 진단 JSONL의 `cartesian_path`에 start/goal/reference horizon,
적용 선속도·각속도 제한, 진행률, line 기준 편차가 기록된다.

Task-space 모드에서는 `cartesian_path_tolerance: 0.005` [m]로 TCP의
시작점–목표점 **선분까지의 최단거리**를 5 mm 이내로 제한한다. 시간별
기준점에 늦게 도착해도 선분 위라면 경로 편차는 0이다. 위치 이동 없이
회전만 할 때에는 고정 위치를 중심으로 같은 반경을 적용한다.
MPC의 running/terminal 비용에 `cartesian_path` 항(가중치 `1/tolerance²`)을
추가하고, 관절 제한을 적용한 최종 q_cmd에는 별도 경로 검사를 적용한다.
경계 근처의 명령은 Jacobian으로 경로 안쪽에 보정하되 관절 위치·속도·증분
제한을 유지한다. 관절 직선 보간 중의 TCP 곡률 상한까지 검사하여 허용 범위에 들어올 때까지
명령 증분을 절반씩 줄이며, 적합한 증분이 없거나 실측 TCP가 이미 범위를
벗어났으면 현재 관절 위치를 유지한다. 이때 자동으로 직선을 재설정하지 않는다.
동일 목표의 반복 수신도 직선을 재설정하지 않으며, 변경된 목표의 선분이
현재 TCP를 허용 범위 안에 포함하지 못하면 현재 실측 TCP에서 새 이동을 시작한다.
진단 `cartesian_path`의 `deviation_m`, `tolerance_m`, `command_scale`, `guard`로
실측 편차와 제한 동작을 확인한다. `cartesian_path_limited`는 보정/축소,
`cartesian_path_hold`는 가능한 증분이 없어 유지, `cartesian_path_violation`은
실측 편차 초과를 뜻한다. `/mpc/solver_success`는 솔버의 수렴 여부이며,
`/mpc/status` (`std_msgs/String`)는 경로 제한·유지·편차 초과와 솔버 상태를
구분한다. Plot은 경로 보정 동작을 `PATH LIMITED`로 표시하며 솔버 실패로
취급하지 않는다. 수렴하지 않은 경우도 목표 도달 불가 판정과는 다르다.
이 검사는 URDF 운동학과 관절 직선 보간에 대한 소프트웨어 제한이다.
실제 로봇의 추종 오차, 하위 드라이버의 별도 관절 필터 및 보간 동작은
포함하지 않으므로 실제 TCP의 5 mm 보장은 실측 검증이 필요하다.
허용 편차 변경은 YAML 수정 후 노드를 재시작해야 적용된다.

키보드로 desired TCP pose를 조작하려면 MPC를 실행한 터미널과 별도의 터미널에서 다음을 실행한다.

```bash
docker exec -it robot-teleoperation bash -lc \
  'source /opt/ros/jazzy/setup.bash && \
   source /opt/robot_teleoperation_framework/install/setup.bash && \
   ros2 run indy7_mpc keyboard_teleop'
```

`w/s`, `a/d`, `r/f`는 각각 world 기준 x/y/z 이동이고, `u/o`, `i/k`, `j/l`은 roll/pitch/yaw 회전이다. `space`는 초기 pose로 복귀하고 `q`는 종료한다. 기본 이동량은 1 cm, 회전량은 5 deg이며 ROS parameter로 조절할 수 있다.

```bash
docker exec robot-teleoperation bash -lc \
  'source /opt/ros/jazzy/setup.bash && \
   source /opt/robot_teleoperation_framework/install/setup.bash && \
   ros2 topic echo /mpc/cost'
```

목표 pose와 실제 pose를 함께 확인하려면:

```bash
docker exec robot-teleoperation bash -lc \
  'source /opt/ros/jazzy/setup.bash && \
   source /opt/robot_teleoperation_framework/install/setup.bash && \
   ros2 topic echo /mpc/desired_pose'

docker exec robot-teleoperation bash -lc \
  'source /opt/ros/jazzy/setup.bash && \
   source /opt/robot_teleoperation_framework/install/setup.bash && \
   ros2 topic echo /mpc/current_pose'

docker exec robot-teleoperation bash -lc \
  'source /opt/ros/jazzy/setup.bash && \
   source /opt/robot_teleoperation_framework/install/setup.bash && \
   ros2 topic echo /mpc/command_pose'

docker exec robot-teleoperation bash -lc \
  'source /opt/ros/jazzy/setup.bash && \
   source /opt/robot_teleoperation_framework/install/setup.bash && \
   ros2 topic echo /mpc/joint_position_cmd'

docker exec robot-teleoperation bash -lc \
  'source /opt/ros/jazzy/setup.bash && \
   source /opt/robot_teleoperation_framework/install/setup.bash && \
   ros2 topic echo /mpc/task_error'
```

실험별 목표·MPC·cost 값은 소스 코드를 수정하지 말고 YAML을 복사해서 관리한다.

```bash
mkdir -p .docker-data
cp ros2_ws/src/indy7_mpc/config/indy7_mpc.yaml \
  .docker-data/my_experiment.yaml
# my_experiment.yaml의 goal_position/goal_orientation_xyzw와 가중치를 수정
docker run --rm --net=host --ipc=host \
  -v "$PWD/.docker-data/my_experiment.yaml:/tmp/my_experiment.yaml:ro" \
  robot-teleoperation-framework:mpc \
  ros2 launch indy7_mpc indy7_mpc.launch.py \
  config_file:=/tmp/my_experiment.yaml
```

기본 목적함수는 다음과 같다.

```text
J = J_SE3 + J_cartesian_path + J_posture + J_velocity + J_torque + J_joint_limit
```

위치/자세 weight는 각각 `20.0/5.0`, posture `0.1`, velocity `1.0`,
torque `0.0001`, joint-limit `10.0`이다. terminal 단계에는
`terminal_cost_weight=10.0`을 적용한다. `joint_limit_margin=0.2 rad` 안쪽을
soft avoidance zone으로 사용하고, 실제 모델의 URDF limit은 시뮬레이션에서
hard clip한다. cost는 정규화되지 않은 horizon 합이므로 horizon/weight를
바꾸면 절대값도 바뀐다.

goal 근처에서 떨리면 `velocity_cost_weight`, `torque_cost_weight`,
`simulation_damping`을 키우고 `se3_*` 또는 `terminal_cost_weight`를 낮춰서
시작한다. 시뮬레이션에서만 적용되는 acceleration smoothing도 조절할 수 있다.

```bash
# my_experiment.yaml에서 다음처럼 조정한 뒤 다시 launch
# se3_position_cost_weight: 12.0
# se3_orientation_cost_weight: 4.0
# ik_damping: 0.003
# ik_step: 0.05
# velocity_cost_weight: 2.0
# torque_cost_weight: 0.001
# terminal_cost_weight: 8.0
# simulation_damping: 8.0
# simulation_acceleration_smoothing: 0.15
```

`torque_cost_weight`를 너무 크게 하면 움직임이 둔해지고,
`se3_*`를 너무 크게 하면 큰 `qddot`가 나올 수 있다. 실제 하드웨어에
연결하기 전에는 반드시 별도의 position/velocity/acceleration 제한과
안전 정지를 추가해야 한다.

기본 다섯 항의 계산과 미분은 [`se3_dynamic_model.py`](src/indy7_mpc/indy7_mpc/se3_dynamic_model.py)에 있다. 실험별 weight는 YAML에서 바꾸고,
새 residual을 추가할 때는 이 action model의 `_terms()`와 `calcDiff()`를
함께 수정해야 한다. `custom_costs.py`는 기존 실험과의 호환을 위해 남겨둔
확장 지점이다. URDF task frame 이름은 `tcp`다. Teleoperation pose 자체는
`teleop/target_pose`에서 실시간으로 갱신된다.

시뮬레이션을 끄고 외부 joint state를 사용하려면:

```bash
docker run --rm --net=host --ipc=host \
  robot-teleoperation-framework:mpc \
  ros2 launch indy7_mpc indy7_mpc.launch.py \
  simulate:=false rviz:=false plot:=false
```

YAML의 초기 SE(3) 목표를 바꾸려면 `goal_position`과 `goal_orientation_xyzw`를 수정한다. Teleoperation 중에는 다음처럼 `PoseStamped`를 발행하면 목표가 실시간으로 바뀐다.

```bash
docker exec robot-teleoperation bash -lc \
  'source /opt/ros/jazzy/setup.bash && \
   source /opt/robot_teleoperation_framework/install/setup.bash && \
   ros2 topic pub --rate 30 /teleop/target_pose geometry_msgs/msg/PoseStamped \
   "{header: {frame_id: world}, pose: {position: {x: 0.55, y: -0.03, z: 0.50}, orientation: {x: -0.02, y: 0.99, z: 0.12, w: 0.10}}}"'
```

## Indy7 실기 teleoperation

계산과 하드웨어 연결은 별도 컨테이너다. MPC는 NumPy 2 / Crocoddyl을, DCP3 adapter는
Neuromeka SDK와 NumPy 1을 사용하므로 같은 Python 환경에 설치하지 않는다.

```bash
make docker-build
make docker-test
```

실제 제어기 IP로 idle 개발 컨테이너를 시작한다. 아래 문자열을 장비의 실제 주소로 바꾼다.
이 명령은 ROS 노드를 launch하지 않으며, 로봇을 arm하거나 움직이지 않는다.

```bash
make docker-up-indy7 ROBOT_IP=controller-address
```

별도 터미널에서 서비스별 shell을 열고 노드를 직접 실행한다. ROS, Python 환경, 마운트된
소스 overlay는 shell 진입 때 자동으로 source된다. 첫 shell 진입 때만 overlay를 준비한다.
일반 Python/launch/YAML 수정은 이미지 빌드 없이 반영되고, package 구조나 의존성 metadata를
바꾼 경우에만 `FORCE_COLCON_BUILD=1`로 해당 overlay를 다시 빌드한다.

터미널 1에서 DCP3 adapter를 실행한다.

```bash
make docker-shell-indy7 ROBOT_IP=192.168.0.110 INDY7_SERVICE=dcp3
ros2 launch indy7_dcp3 indy7_real_teleop.launch.py robot_ip:=$INDY7_ROBOT_IP
```

터미널 2에서 실기 MPC, RViz, 진단 plot을 실행한다. MPC 목표는 첫 실측 상태의 현재 TCP
자세로 초기화된다.

```bash
make docker-shell-indy7 ROBOT_IP=192.168.0.110 INDY7_SERVICE=mpc
ros2 launch indy7_mpc indy7_mpc.launch.py simulate:=false \
  initialize_target_from_current_state:=true rviz:=true plot:=true \
  diagnostic_log_directory:=/tmp/indy7_mpc_logs
```

터미널 3에서 키보드 입력을 현재 TCP 자세에 동기화해 실행한다. 측정 자세는 최초 한 번만
목표에 복사되며, 이후 목표는 키 입력 때만 갱신된다.

```bash
make docker-shell-indy7 ROBOT_IP=192.168.0.110 INDY7_SERVICE=mpc
ros2 run indy7_mpc keyboard_teleop --ros-args -p initialize_from_current_pose:=true
```

터미널 4에서 deadman heartbeat를 발행한다. 10 Hz로 계속 발행해야 하므로 이 명령은
별도 터미널에서 계속 실행해 둔다. 발행이 끊기면 기본 0.3초 timeout 뒤 adapter가 정지 및
disarm 절차를 수행한다. 이 ROS heartbeat는 물리 deadman이나 E-stop이 아니다.

```bash
make docker-shell-indy7 ROBOT_IP=192.168.0.110 INDY7_SERVICE=dcp3
ros2 topic pub -r 10 /indy7_dcp3/deadman std_msgs/msg/Bool "{data: true}"
```

arm 전에는 별도 확인 shell에서 실제 관절 feedback, 현재 TCP, MPC command/목표를 확인한다.
키보드 목표가 현재 자세에서 작은 이동으로 바뀌는지, DCP3가 IDLE인지, 작업공간이 비었는지
확인하고 물리 E-stop에 즉시 접근할 수 있어야 한다.

```bash
make docker-shell-indy7 ROBOT_IP=192.168.0.110 INDY7_SERVICE=dcp3
ros2 topic echo /joint_states --once
ros2 topic echo /indy7_dcp3/deadman --once
```

```bash
make docker-shell-indy7 ROBOT_IP=192.168.0.110 INDY7_SERVICE=mpc
ros2 topic echo /mpc/current_pose --once
ros2 topic echo /mpc/desired_pose --once
ros2 topic echo /mpc/joint_position_cmd --once
```

deadman publisher를 켜 둔 채 **다른 DCP3 shell**에서 arm한다. arm 요청은 DCP3 feedback과 MPC
command가 최신이고, deadman이 살아 있으며, 로봇이 IDLE이고 목표가 제한 안에 있을 때만
수락된다. arm 이후부터 MPC의 절대 관절 목표가 실제 로봇에 전달된다.

```bash
make docker-shell-indy7 ROBOT_IP=192.168.0.110 INDY7_SERVICE=dcp3
ros2 service call /indy7_dcp3/arm std_srvs/srv/SetBool "{data: true}"
```

정상 종료는 **다른 DCP3 shell에서 먼저 명시적으로 disarm**한 뒤 각 launch와 keyboard
프로세스를 `Ctrl+C`로 종료하고, 마지막에 deadman publisher를 `Ctrl+C`로 끝낸다. 그 다음
idle 컨테이너를 종료한다. deadman을 끊으면 watchdog도 정지시키지만, 정상 종료 절차에서는
그것을 disarm 명령 대신 사용하지 않는다.

```bash
make docker-shell-indy7 ROBOT_IP=192.168.0.110 INDY7_SERVICE=dcp3
ros2 service call /indy7_dcp3/arm std_srvs/srv/SetBool "{data: false}"
```

```bash
make docker-down-indy7
```

현재 MPC YAML의 `cartesian_max_speed`는 `0.5 m/s`이고, DCP3 피드백·명령 주기는
200 Hz 설정이다. DCP3의 추가 관절 속도 커팅은 제거했으며 속도·가속도 비율 20%는 유지한다.
속도를 높이는 실험은 `cartesian_max_speed`만 한 번에 한 단계 조정하고,
실제 TCP 응답과 진단 로그를 확인한다. 모든 software limit/watchdog은 안전 정격이 아니며,
물리 E-stop, 충돌 감시, 작업공간 감시를 대체하지 않는다.
