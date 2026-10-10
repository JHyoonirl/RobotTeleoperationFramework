# Indy7 Runtime

이 저장소는 Indy7 실행을 별도 ROS 2 컨테이너로 나눕니다.

- `robot-teleoperation-framework:mpc`: ROS Jazzy, NumPy 2, Pinocchio/Crocoddyl, mock simulation
- `robot-teleoperation-framework:dcp3`: ROS Jazzy, NumPy 1, Neuromeka IndyDCP3 client
- `robot-teleoperation-framework:mpc-gui`: optional RViz and joint-state GUI layer
- `indy7_description`: 두 컨테이너가 공유하는 URDF와 mesh

계산 런타임과 제조사 통신 SDK의 Python 의존성을 분리해 ABI 충돌을 막고, 실기 adapter를
MPC 의존성 변경으로부터 격리합니다. RViz는 필요할 때만 별도 이미지로 추가합니다. 빌드와
테스트는 저장소 루트에서 실행합니다.

```bash
make docker-build
make docker-test
```

## MPC Simulation

컨테이너 셸을 연 뒤 mock hardware 시뮬레이션을 실행합니다.

```bash
make docker-run
```

```bash
ros2 launch indy7_mpc indy7_mpc.launch.py rviz:=false plot:=false
```

ROS 2와 분리된 MPC 드라이런은 다음처럼 실행할 수 있습니다.

```bash
docker run --rm robot-teleoperation-framework:mpc python3 indy7_mpc_test.py
docker run --rm robot-teleoperation-framework:mpc \
  python3 indy7_mpc_test.py --target '0.1,-0.2,0.3,0,0.2,-0.1'
```

## Hardware

두 Indy 컨테이너를 ROS 2 네트워크에서 함께 시작합니다. 실제 제어기 주소를 지정해야
합니다. 이 단계는 DCP3 상태 피드백을 읽고 MPC를 시작할 뿐, 로봇을 arm하거나 움직이지
않습니다.

```bash
make docker-up-indy7 ROBOT_IP=controller-address
```

첫 이동 전 피드백과 현재 TCP pose를 확인하고, 물리 E-stop이 즉시 사용 가능한 상태에서
명시적인 deadman과 arm 절차를 따라야 합니다. 상세 단계는
[`indy7_dcp3 README`](../../ros2_ws/src/indy7_dcp3/README.md)에 있습니다.
