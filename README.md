# Robot Teleoperation Framework

Neuromeka Indy7 조작 및 task-space MPC 실험을 위한 ROS 2 저장소입니다.
현재 저장소에는 다음 범위가 들어 있습니다.

- `indy7_mpc`: Indy7 URDF, Crocoddyl/Pinocchio 기반 SE(3) MPC, RViz 및 mock `ros2_control` 실행
- `manipulator/indy7_mpc_test.py`: ROS 2와 분리된 최소 MPC 드라이런
- `manipulator/requirements-crocoddyl.txt`: Python 기반 MPC 의존성

실제 로봇에 명령을 보내는 IndyDCP3 adapter, deadman/E-stop, 충돌 감시는 포함되어 있지
않습니다. 실기 배포 전에는 반드시 별도 하드웨어 경계를 추가해야 합니다.

## Directory layout

```text
manipulator/
├── indy7_mpc_test.py
├── requirements-crocoddyl.txt
└── ros2_ws/
    ├── README.md
    └── src/
        └── indy7_mpc/
            ├── config/
            ├── launch/
            ├── meshes/
            ├── urdf/
            └── indy7_mpc/
```

`build/`, `install/`, `log/`, `logs/`, `.venv/`는 로컬 생성물이라 Git에 넣지 않습니다.

## Requirements

- Ubuntu 환경의 ROS 2 Jazzy
- Python 3.12 권장
- `colcon`, `rosdep`, `python3-venv`
- Crocoddyl, Pinocchio, NumPy 등은 `manipulator/requirements-crocoddyl.txt`로 설치

## Clone and build

```bash
git clone <repository-url> RobotTeleoperationFramework
cd RobotTeleoperationFramework

export ROS_DISTRO=${ROS_DISTRO:-jazzy}
source "/opt/ros/$ROS_DISTRO/setup.bash"

python3 -m venv --system-site-packages --copies .venv
source .venv/bin/activate
python3 -m pip install -r manipulator/requirements-crocoddyl.txt

rosdep install --from-paths manipulator/ros2_ws/src --ignore-src -r -y
make build
source manipulator/ros2_ws/install/setup.bash
```

이미 정상적인 `.venv`가 있으면 virtualenv 생성 단계는 건너뛰면 됩니다. 다른 경로에서
복사했거나 Python 실행 파일이 시스템 Python을 직접 가리키는 오래된 `.venv`는 삭제 후
위 명령으로 다시 만드십시오. 이후에는 다음처럼 루트에서 반복 작업할 수 있습니다.

```bash
make build
make test
make list
```

## Run the simulation

```bash
source /opt/ros/${ROS_DISTRO:-jazzy}/setup.bash
source .venv/bin/activate
source manipulator/ros2_ws/install/setup.bash
ros2 launch indy7_mpc indy7_mpc.launch.py
```

GUI가 없는 환경에서는 다음처럼 RViz와 plot 창을 끌 수 있습니다.

```bash
ros2 launch indy7_mpc indy7_mpc.launch.py rviz:=false plot:=false
```

세부 토픽, YAML 파라미터, 실기 adapter 경계는
[`manipulator/ros2_ws/README.md`](manipulator/ros2_ws/README.md)를 참고합니다.

## Direct smoke test

```bash
source .venv/bin/activate
python manipulator/indy7_mpc_test.py
```

## Docker

Docker는 ROS 2 Jazzy, `ros2_control` mock hardware, Crocoddyl/Pinocchio와
패키지 테스트를 이미지 빌드 단계에서 함께 설치·검증합니다. 기본 실행은 GUI 없는
시뮬레이션입니다.

```bash
docker build -f docker/Dockerfile -t robot-teleoperation-framework .
docker run --rm --net=host --ipc=host robot-teleoperation-framework
```

빌드된 이미지의 테스트 결과만 다시 확인하려면 다음을 실행합니다.

```bash
make docker-test
```

RViz나 실시간 plot을 사용할 때는 X11/Wayland 전달이 필요하므로, headless 기본값을
유지한 채 필요한 경우 launch 인자를 직접 덮어씁니다.

## Release checklist

1. `make build`와 `make test`가 새 환경에서 통과하는지 확인합니다.
2. 실제 장비 연결 전 joint limit, velocity limit, watchdog, deadman, E-stop을 검증합니다.
3. `ros2_ws/config`의 실험 파라미터와 실제 장비 설정을 분리해 관리합니다.
4. 공개 배포 전 저장소의 최상위 라이선스를 프로젝트 정책에 맞게 결정합니다.
