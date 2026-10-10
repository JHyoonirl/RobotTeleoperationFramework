# Robot Teleoperation Framework

Neuromeka Indy7 조작 및 task-space MPC 실험을 위한 ROS 2 저장소입니다.
이 저장소의 공식 사용 경로는 Docker입니다. 호스트에는 Docker Engine 또는 Docker
Desktop만 준비하면 되며, ROS 2·Python virtualenv·Crocoddyl은 이미지 안에 설치됩니다.

현재 저장소에는 다음 범위가 들어 있습니다.

- `indy7_mpc`: Indy7 URDF, Crocoddyl/Pinocchio 기반 SE(3) MPC, RViz 및 mock `ros2_control` 실행
- `manipulator/indy7_mpc_test.py`: ROS 2와 분리된 최소 MPC 드라이런
- `docker/Dockerfile`: ROS 2 Jazzy 기반 실행·테스트 이미지 빌드 레시피

실제 로봇에 명령을 보내는 IndyDCP3 adapter, deadman/E-stop, 충돌 감시는 포함되어 있지
않습니다. 현재 Docker 실행은 mock hardware 시뮬레이션 검증용이며, 실기 배포 전에는
별도 하드웨어 안전 경계를 추가해야 합니다.

## Directory layout

```text
.
├── docker/
│   ├── Dockerfile
│   └── entrypoint.sh
├── Makefile
├── manipulator/
│   ├── indy7_mpc_test.py
│   ├── requirements-crocoddyl.txt
│   └── ros2_ws/
│       ├── README.md
│       └── src/indy7_mpc/
│           ├── config/
│           ├── launch/
│           ├── meshes/
│           ├── urdf/
│           └── indy7_mpc/
└── README.md
```

`build/`, `install/`, `log/`, `logs/`, `.venv/`는 생성물이며 Git에 넣지 않습니다.
Docker 이미지도 GitHub에 업로드하지 않습니다. Git에는 이미지를 재현하는
`docker/Dockerfile`과 컨테이너 초기화를 담당하는 `docker/entrypoint.sh`만 포함합니다.

## Requirements

- Docker Engine 또는 Docker Desktop
- Docker BuildKit/buildx 권장
- headless 실행은 Linux Docker host 기준으로 검증했습니다. Docker Desktop에서는 `--net=host` 지원 여부와 DDS 네트워크 설정을 확인해야 합니다.
- RViz 같은 GUI를 사용할 때는 호스트 운영체제에 맞는 X11/Wayland 전달 설정이 추가로 필요합니다.

호스트의 ROS 2, `rosdep`, `python3-venv`, Crocoddyl 설치는 필요하지 않습니다.

## Clone and build

```bash
git clone https://github.com/JHyoonirl/RobotTeleoperationFramework.git RobotTeleoperationFramework
cd RobotTeleoperationFramework

make docker-build
```

같은 작업을 Docker 명령으로 직접 실행할 수도 있습니다.

```bash
docker build -f docker/Dockerfile -t robot-teleoperation-framework .
```

이미지 빌드 단계에서 ROS 패키지를 컴파일하고 `indy7_mpc` 패키지 테스트까지 실행합니다.
이미지는 ROS/RViz 및 Python 과학 계산 의존성을 포함하므로 로컬 Docker 저장소에서 수 GB가
될 수 있습니다. 이 크기는 Git 저장소나 Dockerfile의 크기가 아닙니다.

## Verify the image

```bash
make docker-test
```

또는 컨테이너 안에서 테스트 결과를 직접 확인합니다.

```bash
docker run --rm robot-teleoperation-framework \
  python3 -m colcon test-result --verbose
```

ROS와 분리된 MPC 드라이런도 이미지 안에서 실행합니다.

```bash
docker run --rm robot-teleoperation-framework \
  python3 indy7_mpc_test.py

docker run --rm robot-teleoperation-framework \
  python3 indy7_mpc_test.py \
  --target '0.1,-0.2,0.3,0,0.2,-0.1'
```

## Run the simulation

기본 명령은 GUI 없는 headless 시뮬레이션입니다.

```bash
make docker-run
```

직접 실행할 때는 다음과 같습니다. `--net=host`와 `--ipc=host`는 ROS 2 DDS 통신을
호스트와 맞추기 위한 옵션입니다.

```bash
docker run --rm --net=host --ipc=host \
  robot-teleoperation-framework
```

실행 옵션을 바꾸려면 이미지의 기본 명령을 덮어씁니다.

```bash
docker run --rm --net=host --ipc=host \
  robot-teleoperation-framework \
  ros2 launch indy7_mpc indy7_mpc.launch.py rviz:=false plot:=false
```

컨테이너를 백그라운드에서 유지하면서 토픽을 확인하려면 이름을 지정합니다.

```bash
docker run -d --name robot-teleoperation --net=host --ipc=host \
  robot-teleoperation-framework

docker logs -f robot-teleoperation
```

다른 터미널에서 컨테이너 내부의 ROS 명령을 실행할 수 있습니다.

```bash
docker exec robot-teleoperation bash -lc \
  'source /opt/ros/jazzy/setup.bash && \
   source /opt/robot_teleoperation_framework/install/setup.bash && \
   ros2 topic list'
```

종료 후 컨테이너를 정리합니다.

```bash
docker rm -f robot-teleoperation
```

## Detailed documentation

- [Indy7 MPC 시작점](manipulator/README.md)
- [ROS 2 패키지, 토픽, 파라미터 및 실험 방법](manipulator/ros2_ws/README.md)

## Hardware boundary

실기 연결은 다음 단계에서 IndyDCP3 client를 사용하는 별도 adapter로 추가해야 합니다.
adapter에는 최소한 joint/velocity/acceleration limit, 통신 watchdog, deadman, 충돌 감시,
비상정지(E-stop)를 포함하고, `mpc/joint_position_cmd`를 실제 로봇에 전달하기 전에
안전 검사를 수행해야 합니다.

## Release checklist

1. `make docker-build`가 새 환경에서 성공하는지 확인합니다.
2. `make docker-test`와 headless 시뮬레이션이 통과하는지 확인합니다.
3. 실제 장비 연결 전 하드웨어 adapter의 limit, watchdog, deadman, E-stop을 검증합니다.
4. 공개 배포 전 저장소의 최상위 라이선스를 프로젝트 정책에 맞게 결정합니다.
