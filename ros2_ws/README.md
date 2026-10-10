# Neuromeka Indy7 + ROS2 + Crocoddyl

공식 `neuromeka-robotics/indy-ros2`에서 Indy7에 필요한 것만 가져왔다.

- 공식 생성 URDF: `src/indy7_mpc/urdf/indy7.urdf`
- Indy7 링크의 visual/collision STL 14개
- ROS2 description/시각화와 Crocoddyl MPC를 합친 단일 최소 패키지
- `indy_driver`, `indy_gazebo`, `indy_moveit`, `indy_interfaces`는 복사하지 않음

Indy7은 제품명이 7이지만 실제 관절은 6개다. 기본 MPC 상태는
`x=[q, qdot]`, 내부 최적화 입력은 URDF effort limit 안의 관절 토크 `u=tau`다.
Pinocchio ABA가 질량행렬·중력·코리올리 항을 반영해 다음 상태를 예측한다.
매 주기 FDDP가 목적함수를 풀고 첫 번째 예측 상태의 `q_cmd`를
라디안 절대 관절 위치로 출력한다. 시뮬레이션에서는 이 토픽을
`forward_position_controller`에 연결하고, 실기에서는 같은 토픽을
IndyDCP3 `TELE_JOINT_ABSOLUTE / movetelej_abs()` adapter에 연결한다.

## Docker 실행환경

이 문서의 모든 ROS 2 명령은 Docker 컨테이너 안에서 실행한다. 호스트에서 ROS 2를
설치하거나 `.venv`를 만들거나 `rosdep`를 실행하지 않는다. 의존성 설치, colcon build,
패키지 테스트는 저장소 루트의 `docker/Dockerfile`에 고정되어 있다.

저장소 루트에서 이미지를 빌드하고 테스트한다.

```bash
cd /path/to/RobotTeleoperationFramework
make docker-build
make docker-test
```

이미지 안에는 다음 경로로 빌드된 ROS 2 workspace가 있다.

```text
/opt/robot_teleoperation_framework/src
/opt/robot_teleoperation_framework/install
```

기본 이미지 이름은 `robot-teleoperation-framework`이며, 다른 이름을 사용하면 이후
명령의 이미지 이름도 동일하게 바꾼다.

## 실행

```bash
docker run --rm --net=host --ipc=host \
  robot-teleoperation-framework
```

위 명령은 이미지에 정의된 headless 기본 launch를 실행한다. launch 인자를 바꾸려면
이미지의 기본 명령을 덮어쓴다.

```bash
docker run --rm --net=host --ipc=host \
  robot-teleoperation-framework \
  ros2 launch indy7_mpc indy7_mpc.launch.py rviz:=false plot:=false
```

토픽 확인이나 로그 분석을 위해 컨테이너를 백그라운드에서 유지하려면 이름을 지정한다.

```bash
docker run -d --name robot-teleoperation --net=host --ipc=host \
  robot-teleoperation-framework
docker logs -f robot-teleoperation
```

이후의 `docker exec robot-teleoperation ...` 명령은 이 컨테이너가 실행 중이라는 전제다.

기본값은 `config/indy7_mpc.yaml`을 읽는 시뮬레이션 모드이며, `tcp` frame의 SE(3) 목표를 직접 추적한다. DLS IK는 `qref` 진단용으로만 남아 있으며,
MPC의 tracking residual은 Pinocchio forward kinematics로 매 stage 계산된다.
`ros2_control_node`가 `mock_components/GenericSystem`과 함께 시작되고,
`joint_state_broadcaster` 및 `forward_position_controller`가 활성화된다.
MPC는 매 주기 `mpc/joint_position_cmd`에 `Float64MultiArray([q0..q5])`를
발행하며 launch에서 `/forward_position_controller/commands`로 remap한다.
Task-space 목표는 horizon의 모든 단계에서 같은 최종 pose를 추적하지 않고,
현재 TCP에서 최종 목표까지의 직선 위치 기준과 SO(3) 방향 보간 기준으로
분해된다. 기준 속도는 `cartesian_max_speed`(기본 0.05 m/s)와
`cartesian_max_angular_speed`(기본 0.75 rad/s)로 각각 제한되며,
현재 Jacobian과 URDF joint-velocity limit의 50%를 넘지 않도록 자동으로
낮아진다. `cartesian_reference_topic`에서 현재 horizon 첫 번째 기준 pose를
확인할 수 있다.
RViz 설정(`config/indy7.rviz`)은 로봇 Alpha를 `0.5`로 표시하고 `tcp` TF 기준의 TCP Axes display를 항상 켠다.
launch를 실행하면 `mpc_plotter`가 함께 열려 position/orientation 오차, 전체 cost, solver 성공 여부·계산시간, control norm, 최소 joint-limit 여유를 실시간으로 표시한다. GUI가 없는 환경에서는 `plot:=false`로 끌 수 있다.

동시에 진단 로그가 기본적으로 컨테이너 내부의
`/opt/robot_teleoperation_framework/logs/indy7_mpc/<UTC timestamp>/trace.jsonl`에 저장된다.
각 줄은 한 MPC 주기의 JSON record이며 현재/목표 pose, local SE(3) 오차, q/qdot/qref,
hard·soft joint-limit 여유, raw torque와 ABA 동역학 항, cost 항, solver 결과, TCP Jacobian의
singular value/condition number, 월드 Z 회전에 필요한 추정 qdot, teleop pose 갱신 주기와
목표 각속도, joint-state age, 전체 예측 `xs/us`를 포함한다.
같은 디렉터리의 `parameters.json`은 해당 실행의 파라미터 snapshot이다.
기록을 끄거나 저장 위치를 바꾸려면 YAML의 `diagnostic_logging`,
`diagnostic_log_directory`, `diagnostic_log_prediction`을 수정한다.
`--rm`으로 컨테이너를 제거하면 컨테이너 내부 로그도 함께 사라지므로 보존하려면
앞의 백그라운드 실행 명령 대신 호스트 디렉터리를 로그 경로에 bind mount한다.

```bash
mkdir -p .docker-data/logs
docker run -d --name robot-teleoperation --net=host --ipc=host \
  -v "$PWD/.docker-data/logs:/opt/robot_teleoperation_framework/logs" \
  robot-teleoperation-framework
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

입력 pose의 `header.frame_id`는 기본적으로 `world`여야 한다. 다른 frame을 쓰려면 먼저 TF 변환을 추가하거나 `pose_frame`을 맞춘다. `q_cmd`에는 URDF hard joint limit, measured-q 기준 velocity limit, cycle increment limit을 모두 적용한다. 실기에서는 `mpc/joint_position_cmd`를 IndyDCP3 adapter의 입력으로 사용하고, adapter에서 rad→deg 변환 및 deadman/E-stop을 추가한다. 실행 중 Crocoddyl 전체 목적함수 값은 다음 토픽에서 확인할 수 있다.

Cartesian 속도 관련 YAML 파라미터는 다음과 같다.

```yaml
cartesian_max_speed: 0.05       # m/s, 사용자가 정하는 상한
cartesian_max_angular_speed: 0.75 # rad/s, orientation 상한
cartesian_speed_margin: 0.5     # joint velocity limit의 사용 비율
cartesian_min_duration: 0.10    # 너무 짧은 목표 이동 방지 [s]
```

실제 위치 기준 속도는 `min(cartesian_max_speed, Jacobian 기반 joint-velocity cap)`이고,
orientation 기준은 `cartesian_max_angular_speed`에 의해 별도로 제한된다.
따라서 특이점 근처에서는 자동으로 느려질 수 있다. teleoperation pose가 연속해서
들어오는 동안에는 path 시작점을 매번 재설정하지 않고, 현재 line의 목표점만
갱신한다. 진단 JSONL의 `cartesian_path`에 start/goal/reference horizon,
적용 선속도·각속도 제한, 진행률, line 기준 편차가 기록된다.

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
  robot-teleoperation-framework \
  ros2 launch indy7_mpc indy7_mpc.launch.py \
  config_file:=/tmp/my_experiment.yaml
```

기본 목적함수는 다음과 같다.

```text
J = J_SE3 + J_posture + J_velocity + J_torque + J_joint_limit
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
  robot-teleoperation-framework \
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

실기 연결은 다음 단계에서 IndyDCP3 client를 사용하는 별도 adapter를 붙인다. adapter는 `mpc/joint_position_cmd`를 구독해 `TELE_JOINT_ABSOLUTE` 모드의 `movetelej_abs()`로 전달한다. `q_cmd`는 이미 위치·속도·증분 제한을 통과했지만 실제 joint limits, 통신 watchdog, deadman, 충돌 감시, E-stop은 adapter/robot side에서 추가로 확인해야 한다.
