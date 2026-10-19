# Robot Teleoperation Framework

Neuromeka Indy7 조작 및 task-space MPC 실험을 위한 ROS 2 저장소입니다.
이 저장소의 공식 사용 경로는 Docker입니다. 호스트에는 Docker Engine 또는 Docker
Desktop만 준비하면 되며, ROS 2와 MPC 의존성은 이미지 안에 설치됩니다.

기본 빌드는 Indy7 MPC와 Vive Tracker ROS 2 패키지를 사용합니다. MANUS-WUJI 손
리타게팅은 같은 workspace에 배치된 선택형 Git submodule입니다.

- `indy7_description`: Indy7 URDF와 mesh 등 공용 로봇 모델
- `indy7_mpc`: Crocoddyl/Pinocchio 기반 SE(3) MPC, RViz 및 mock `ros2_control` 실행
- `indy7_dcp3`: MPC 관절 명령과 Neuromeka IndyDCP3 실기 연결
- `vive_tracker_ros2`: Vive Tracker 입력 ROS 2 패키지
- `hand_retargetting`: MANUS-WUJI 입력/retargetting ROS 패키지 및 독립 Docker 런타임
- `robots/indy7/indy7_mpc_test.py`: ROS 2와 분리된 최소 MPC 드라이런
- `docker/Dockerfile`: MPC와 IndyDCP3 전용 이미지 빌드 레시피

IndyDCP3 연결에는 명시적 arm 서비스, deadman/명령 timeout, URDF 관절 제한 및 보수적인
관절 setpoint 변화 제한이 적용됩니다. 이는 안전 인증 기능이나 안전 정격 E-stop을 대체하지 않으므로
실기 테스트 중에는 로봇의 물리 E-stop을 사용할 수 있어야 합니다.

## Directory layout

```text
.
├── docker/
│   ├── Dockerfile
│   ├── compose.indy7.yaml
│   ├── dev-shell.sh
│   └── entrypoint.sh
├── Makefile
├── robots/
│   └── indy7/
│       ├── README.md
│       ├── indy7_mpc_test.py
│       ├── requirements-mpc.txt
│       └── requirements-hardware.txt
├── ros2_ws/
│   ├── README.md
│   └── src/
│       ├── indy7_description/     # 공용 URDF와 mesh
│       ├── indy7_mpc/             # MPC 계산 패키지
│       ├── indy7_dcp3/            # IndyDCP3 실기 adapter
│       ├── vive_tracker_ros2/     # Vive 입력 (git submodule)
│       └── hand_retargetting/     # 선택형 Hand 저장소 (git submodule)
│           ├── COLCON_IGNORE
│           └── packages/         # MANUS, retargeting, WUJI 패키지
└── README.md
```

`ros2_ws/build/`, `ros2_ws/install/`, `ros2_ws/log/`, `ros2_ws/logs/`, `.venv/`는
생성물이며 Git에 넣지 않습니다.
Docker 이미지도 GitHub에 업로드하지 않습니다. Git에는 이미지를 재현하는
`docker/Dockerfile`과 컨테이너 초기화를 담당하는 `docker/entrypoint.sh`만 포함합니다.

## Requirements

- Docker Engine 또는 Docker Desktop
- Docker BuildKit/buildx와 Compose plugin
- 실기 Indy7 개발 실행은 Linux 데스크톱의 X11/XWayland 세션이 필요합니다. Docker Desktop에서는 `--net=host`, DDS, GUI 전달이 다를 수 있습니다.

호스트에 ROS 2나 Python MPC 의존성을 설치할 필요가 없습니다. MPC는 NumPy 2 계열의
Pinocchio/Crocoddyl 환경이고, DCP3는 Neuromeka SDK가 요구하는 NumPy 1 계열 환경입니다.
두 Python ABI를 섞지 않도록 별도 이미지와 컨테이너로 빌드·실행합니다. MANUS/WUJI도
독립 이미지로 관리해 장치 권한과 SDK 수명주기를 분리합니다.

## Clone and build

```bash
git clone --recurse-submodules https://github.com/JHyoonirl/RobotTeleoperationFramework.git RobotTeleoperationFramework
cd RobotTeleoperationFramework

make docker-build

# Build the independent MANUS/WUJI image too
make docker-build INCLUDE_HAND=1
```

`make docker-build`는 `robot-teleoperation-framework:mpc`와
`robot-teleoperation-framework:dcp3`를 빌드합니다. `INCLUDE_HAND=1`을 추가하면
Hand 저장소의 전용 Dockerfile도 사용해 Hand 이미지를 별도로 빌드합니다.
Indy MPC와 DCP3 이미지는 `docker/compose.indy7.yaml`에서 ROS 2 네트워크로 연결됩니다.
RViz가 포함된 `mpc-gui` 이미지가 없으면 `make docker-up-indy7` 실행 시 자동으로 빌드됩니다.

이미지를 직접 빌드할 수도 있습니다.

```bash
docker buildx build --load \
  --target mpc -f docker/Dockerfile \
  -t robot-teleoperation-framework:mpc .
docker buildx build --load \
  --target dcp3 -f docker/Dockerfile \
  -t robot-teleoperation-framework:dcp3 .
```

이미지 빌드 단계에서 해당 ROS 패키지를 컴파일하고 테스트합니다. MPC 이미지에는
Pinocchio/Crocoddyl 계산 런타임, DCP3 이미지에는 Neuromeka 통신 SDK만 들어갑니다.

## Make 명령과 실행 흐름

기본 사용 경로는 Docker입니다. `build`, `test`, `list`, `run-mpc` 호스트 타깃은 ROS 2
Jazzy와 저장소의 `.venv`를 사용합니다. `build-hand`는 호스트 ROS/colcon 환경과 Hand
의존성을 사용합니다. Docker 타깃은 호스트 Docker Engine과 Compose/buildx 설정을 사용합니다.

| 명령 | 동작 |
| --- | --- |
| `make build` | 호스트에서 `ros2_ws/src`의 ROS 패키지를 빌드합니다. |
| `make build-hand` | 호스트에서 Hand 패키지를 빌드합니다. ROS 환경이 필요하며 WUJI SDK가 없으면 WUJI 드라이버 패키지는 건너뜁니다. |
| `make test` | 호스트에서 ROS 패키지 테스트를 실행합니다. |
| `make list` | colcon이 찾은 ROS 패키지를 표시합니다. |
| `make run-mpc` | 호스트 workspace의 `indy7_mpc` launch를 실행합니다. |
| `make docker-build` | MPC와 DCP3 이미지를 각각 빌드합니다. Hand 이미지는 포함하지 않습니다. |
| `make docker-build INCLUDE_HAND=1` | MPC, DCP3를 빌드한 뒤 Hand 이미지도 별도로 빌드합니다. 총 3개 이미지입니다. |
| `make docker-build-dcp3` | Indy 실기 연결용 DCP3 이미지만 빌드합니다. |
| `make docker-build-hand` | MANUS-WUJI Hand 이미지 하나만 빌드합니다. Indy 이미지는 빌드하지 않습니다. |
| `make docker-build-gui` | RViz와 Qt teleoperation GUI에 필요한 별도 `mpc-gui` 이미지를 빌드합니다. |
| `make docker-test` | 이미 빌드된 MPC와 DCP3 이미지의 colcon 테스트 결과를 표시합니다. |
| `make docker-run` | 저장소 소스를 마운트하고 source overlay를 준비해 셸을 엽니다. ROS launch는 직접 실행합니다. |
| `make docker-run-gui` | GUI 지원 이미지가 없을 때만 빌드하고, 마운트된 source overlay로 셸을 엽니다. |
| `make docker-up-hand` | MANUS USB 장치를 확인하고 Hand 컨테이너에서 MANUS publisher만 백그라운드로 시작합니다. |
| `make docker-down-hand` | Hand Compose 컨테이너를 종료하고 정리합니다. |
| `make docker-check-indy7 ROBOT_IP=<주소>` | DCP3 연결/feedback 확인용 컨테이너만 실행합니다. MPC는 시작하지 않습니다. |
| `make docker-up-indy7 ROBOT_IP=<주소>` | GUI 지원 MPC/DCP3 개발 컨테이너만 백그라운드로 준비합니다. ROS 노드는 자동 실행하지 않습니다. |
| `make docker-shell-indy7 ROBOT_IP=<주소> INDY7_SERVICE=<service>` | `INDY7_SERVICE`를 `mpc` 또는 `dcp3`로 지정해 해당 컨테이너 shell에 들어갑니다. |
| `make docker-down-indy7` | Indy7 Compose 컨테이너를 종료하고 정리합니다. |
| `make docker-up-indy7-sim` | 실제 IP 없이 MPC 시뮬레이션 컨테이너만 시작합니다. DCP3 컨테이너는 시작하지 않습니다. |
| `make docker-launch-indy7-sim` | mock ros2_control 시뮬레이션, RViz, MPC plot, teleoperation GUI를 실행합니다. |
| `make docker-launch-indy7-sim-minimal` | 기존 GUI는 유지하고, 조인트 매핑과 TCP 키보드 제어에 집중한 최소 GUI로 시뮬레이션을 실행합니다. |
| `make docker-keyboard-indy7-sim` | 별도 터미널에서 키보드 TCP teleoperation을 실행합니다. |
| `make docker-down-indy7-sim` | simulation MPC 컨테이너를 중지합니다. |

컨테이너에는 저장소가 `/workspace`로 마운트되고, MPC 패키지는 이 마운트된 소스에서
`--symlink-install`로 실행됩니다. 첫 진입 때만 colcon overlay를 `.docker-data/indy7-mpc`에
준비합니다. 이후 Python, launch, RViz config 등의 수정은 이미지 재빌드 없이 반영됩니다.
새 파일이나 `package.xml`/`setup.py` 같은 패키지 구성 변경으로 다시 빌드해야 할 때는
`make docker-run-gui FORCE_COLCON_BUILD=1`을 사용하세요. `docker-build-gui`는 GUI/시스템
의존성이나 Dockerfile 변경 시 이미지 자체를 갱신할 때 사용합니다. 기본 MPC 시뮬레이션은
셸에서 직접 launch하며 `simulate`의 기본값은 `true`입니다. 소스 편집은 실행 중인 Python
프로세스에 자동 반영되지 않으므로, 수정한 노드나 launch 코드는 MPC를 재시작해야 적용됩니다.

새 최소 teleoperation GUI는 `FORCE_COLCON_BUILD=1 make docker-launch-indy7-sim-minimal`로
실행합니다. 기존 전체 GUI는 `make docker-launch-indy7-sim` 타깃에서 그대로 사용할 수 있습니다.

MPC cost 항목은 예외로, 실행 중인 노드가 로컬 마운트 파일을 0.25초 간격으로 확인합니다.
호스트에서 다음 파일을 저장하면 컨테이너 안의 같은 파일이 즉시 보이고, 유효한 cost 변경은
현재 solver에 다시 반영됩니다. YAML을 잠시 잘못된 상태로 저장해도 마지막 유효 설정을 유지하고,
파일을 고친 뒤 다시 저장하면 적용됩니다.

```text
ros2_ws/src/indy7_mpc/config/indy7_mpc.yaml
```

실행 중 바꿀 수 있는 항목은 `se3_position_cost_weight`, `se3_orientation_cost_weight`,
`posture_cost_weight`, `velocity_cost_weight`, `torque_cost_weight`,
`joint_limit_cost_weight`, `running_cost_weight`, `terminal_cost_weight`, `nominal_posture`입니다.
`horizon`, `dt`, solver 반복 횟수, 토픽과 제어 모드처럼 solver 구조나 노드 시작 시 설정되는 항목은
재시작해야 적용됩니다. 실행 로그에 `Applied live MPC cost YAML update`가 나오면 새 cost가
적용된 것입니다.

```bash
make docker-run-gui
# 컨테이너 셸에서
ros2 launch indy7_mpc indy7_mpc.launch.py simulate:=true rviz:=true plot:=true \
  diagnostic_log_directory:=/tmp/indy7_sim_logs
```

키보드 TCP teleoperation은 실제 로봇 주소나 DCP3 드라이버 없이 아래처럼 실행할 수 있습니다.
먼저 시뮬레이션 컨테이너를 준비하고, 첫 번째 터미널에서 MPC/RViz를 실행한 다음 두 번째 터미널에서
키보드 노드를 실행합니다.

```bash
make docker-up-indy7-sim
```

터미널 1:

```bash
make docker-launch-indy7-sim
```

터미널 2:

```bash
make docker-keyboard-indy7-sim
```

키는 `w/s=world X`, `a/d=world Y`, `r/f=world Z`, `u/o=roll`, `i/k=pitch`, `j/l=yaw`,
`space=시작 TCP로 복귀`, `q=키보드 노드 종료`입니다. 기본 위치 스텝은 1 cm, 회전 스텝은 5°이며,
키를 누르면 목표 TCP가 해당 스텝만큼 바뀝니다. RViz에서 로봇과 target pose를 보면서 시험하고,
종료할 때는 두 터미널에서 `Ctrl+C`를 누른 뒤 다음 명령으로 MPC 컨테이너를 중지합니다.

```bash
make docker-down-indy7-sim
```

같은 시뮬레이션 launch를 두 번 실행하면 controller manager가 충돌하므로, 실행 중에는
`make docker-launch-indy7-sim`을 다시 실행하지 마세요. launch 타깃은 중복 실행을 거부합니다.
완전히 재시작하려면 `docker-down-indy7-sim`으로 MPC 컨테이너를 멈춘 뒤 `docker-up-indy7-sim`을
실행하고 launch와 keyboard 명령을 다시 시작하세요.

별도 터미널에서 launch가 실행 중일 때 시뮬레이터와 MPC 출력도 확인할 수 있습니다.

```bash
docker exec -it robot-teleoperation bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 topic echo /joint_states --once'
docker exec -it robot-teleoperation bash -lc \
  'source /opt/ros/jazzy/setup.bash && ros2 topic echo /forward_position_controller/commands --once'
```

두 토픽에 6관절 데이터가 나오면 simulation state와 MPC command 경로가 동작 중입니다.
launch는 `Ctrl+C`로 종료합니다.

### 시뮬레이션 자세를 DCP3 실제 로봇에 보내는 GUI

GUI는 관절 이름 순서, 회전 방향, 영점 오프셋(도)을 편집하고 변환된 실제 관절 목표와
측정값을 나란히 보여줍니다. `실제 자세 → 시뮬레이션 동기화`를 누르면 실측 관절값을
역매핑해 MPC에 전달하고, MPC가 그 관절 자세의 FK로 Desired TCP pose를 설정합니다.
이후 시뮬레이션의 현재 관절값을 매핑해 DCP3 명령으로 보냅니다. 초기 매핑은 관절별
1:1, 방향 +1, 오프셋 0도이며 설정은 `.docker-data/indy7-mpc/profiles/indy7_joint_mapping.yaml`에 저장됩니다.

GUI 이미지에는 PyQt5가 필요하므로 한 번 빌드하거나 갱신합니다. 새 GUI 실행 항목과
package metadata를 overlay에 등록하기 위해 기존 overlay가 있으면 첫 실행에 강제 빌드합니다.

```bash
make docker-build-gui
make docker-up-indy7 ROBOT_IP=controller-address
```

터미널 1에서 DCP3 피드백 adapter를 시작합니다.

```bash
make docker-shell-indy7 ROBOT_IP=controller-address INDY7_SERVICE=dcp3
ros2 launch indy7_dcp3 indy7_real_teleop.launch.py robot_ip:=$INDY7_ROBOT_IP
```

터미널 2에서 MPC 시뮬레이션과 GUI를 시작합니다. `make docker-up-indy7`이 MPC와 DCP3
개발 컨테이너를 모두 준비했으므로, 아래 실행 명령으로 mock 시뮬레이션만 launch됩니다.
이미 만들어 둔 overlay를 사용 중이면 `FORCE_COLCON_BUILD=1`을 첫 실행에 추가합니다.

```bash
make docker-launch-indy7-sim FORCE_COLCON_BUILD=1
```

GUI에서 실제 관절 매핑을 확인·저장하고, **실제 자세 → 시뮬레이션 동기화**를 눌러
시뮬레이션이 실측 자세에 도달하는지 확인합니다. Desired TCP pose는 이 자세에서
계산됩니다. 목표를 XYZ·회전 버튼으로 조정한 다음, **매핑된 시뮬레이션 목표 전송 허용**을
켜고 **Deadman 버튼을 누른 채 Arm**을 선택합니다. 시작 목표와 실측 로봇의 최대 관절
오차가 기본 10°보다 크면 GUI에서 Arm을 막습니다. 매핑 오프셋을 조정하거나 다시
동기화한 뒤 시작하세요. Deadman을 놓거나 창 포커스를 잃으면 adapter가 정지·disarm합니다.
창을 닫으면 disarm 서비스를 요청합니다.

정상 종료는 GUI의 **Disarm / 정지**를 누른 뒤 GUI와 DCP3 launch를 `Ctrl+C`로 종료하고,
`make docker-down-indy7`을 실행합니다. `make docker-up-indy7-sim`만 사용하면 DCP3
adapter가 시작되지 않으므로 실제 로봇 연결은 수행되지 않습니다.

MANUS-WUJI는 독립 이미지와 컨테이너를 사용합니다. 다음 명령은 Hand 이미지와 MANUS
publisher를 준비하지만, WUJI 드라이버와 retargeting launch는 자동으로 시작하지 않습니다.
Hand submodule과 LFS SDK를 먼저 준비해야 합니다.

```bash
make docker-build-hand
make docker-up-hand
```

그 다음 별도 호스트 터미널에서 WUJI 드라이버를 실행하고, 또 다른 터미널에서 MANUS-WUJI
retargeting을 실행합니다. `<WUJI_HAND_SERIAL>`은 실제 장비의 시리얼 번호로 바꿉니다.
두 명령은 실행 중인 같은 컨테이너에 노드를 추가합니다.

```bash
docker exec -it manus-retargetting bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /work/ros2_ws/install/setup.bash && ros2 launch wujihand_bringup wujihand.launch.py hand_name:=hand_0 serial_number:=<WUJI_HAND_SERIAL> rviz:=false'

docker exec -it manus-retargetting bash -lc \
  'source /opt/ros/jazzy/setup.bash && source /work/ros2_ws/install/setup.bash && ros2 launch manus_wuji_retargeting manus_wuji_retargeting.launch.py side:=right hand_name:=hand_0'
```

Hand 컨테이너 로그는 `docker logs -f manus-retargetting`로 확인합니다. 종료는
`make docker-down-hand`입니다. Indy MPC/DCP3와 MANUS/WUJI 컨테이너는 모두 `ROS_DOMAIN_ID=77`을
사용합니다. 호스트에서 ROS 노드를 직접 실행할 때도 `export ROS_DOMAIN_ID=77`을 설정해야 하며,
실제 손 구동 전 드라이버와 장치 상태를 확인하십시오.

연결만 빠르게 확인할 때는 이 one-shot 명령을 쓸 수 있습니다. 정상 개발 흐름에서는 아래처럼
idle 컨테이너에 들어가 각 노드를 직접 실행합니다.

```bash
make docker-build-dcp3
make docker-check-indy7 ROBOT_IP=192.168.0.10
```

일반 개발 세션은 컨테이너만 시작합니다. 어떤 ROS 노드도 자동 launch하지 않습니다.
MPC/DCP3 소스는 `/workspace`에서 직접 마운트하며, 각각의 symlink overlay는
`.docker-data/indy7-mpc`, `.docker-data/dcp3`에 보존됩니다. 일반 코드 수정은 이미지 빌드가
필요 없고, 패키지 구성 변경 뒤에는 해당 셸을 강제 재빌드로 다시 엽니다.

```bash
make docker-up-indy7 ROBOT_IP=192.168.0.110
```

터미널 1에서 DCP3 shell을 열고 드라이버를 launch합니다. ROS와 마운트된 source overlay는
진입 시 자동으로 source됩니다.

```bash
make docker-shell-indy7 ROBOT_IP=192.168.0.110 INDY7_SERVICE=dcp3
# 첫 진입 때 overlay를 만들고, 소스 수정은 해당 노드를 재시작하면 반영됩니다.
ros2 launch indy7_dcp3 indy7_real_teleop.launch.py robot_ip:=$INDY7_ROBOT_IP
```

터미널 2에서 MPC launch를 실행합니다. 이 launch가 RViz와 MPC plot을 띄웁니다.

```bash
make docker-shell-indy7 ROBOT_IP=192.168.0.110 INDY7_SERVICE=mpc
ros2 launch indy7_mpc indy7_mpc.launch.py simulate:=false \
  initialize_target_from_current_state:=true rviz:=true plot:=true \
  diagnostic_log_directory:=/tmp/indy7_mpc_logs
```

터미널 3에서 MPC 컨테이너에 다시 들어가 키보드 노드만 직접 실행합니다.

```bash
make docker-shell-indy7 ROBOT_IP=192.168.0.110 INDY7_SERVICE=mpc
ros2 run indy7_mpc keyboard_teleop --ros-args -p initialize_from_current_pose:=true
```

ROS package는 이미지 복사본이 아니라 마운트된 workspace overlay에서 source됩니다. Python,
launch, config 수정은 symlink를 통해 다음 프로세스/launch 시작 때 반영됩니다. package 구조나
의존성 metadata가 바뀌면 관련 컨테이너에 강제 재빌드로 다시 진입합니다.

```bash
make docker-shell-indy7 ROBOT_IP=192.168.0.110 INDY7_SERVICE=mpc FORCE_COLCON_BUILD=1
make docker-shell-indy7 ROBOT_IP=192.168.0.110 INDY7_SERVICE=dcp3 FORCE_COLCON_BUILD=1
```

`make docker-build-gui` 또는 `make docker-build-dcp3`는 Dockerfile, OS package, Python 의존성을
변경했을 때 이미지를 갱신하기 위한 명령이며 일반 ROS 소스 수정에는 필요 없다.

키는 `w/s=x`, `a/d=y`, `r/f=z`, `u/o=roll`, `i/k=pitch`, `j/l=yaw`, `space=목표 초기화`,
`q=종료`입니다. 측정 TCP는 시작 시 한 번만 목표로 복사되고 이후에는 키 입력 때만 목표가
갱신됩니다. 키보드/MPC 실행만으로 실제 로봇을 arm하지 않습니다. 실기 구동 전 피드백,
RViz의 실제/목표 TCP, 작업 공간을 확인하고 아래 [Hardware boundary](#hardware-boundary)의
deadman 및 명시적 arm 순서를 따르십시오.

```bash
make docker-down-indy7
```

종료할 때는 각 launch 터미널에서 `Ctrl+C`를 누른 다음 컨테이너를 내립니다. `ROBOT_IP`는
실제 Indy7 제어기 주소로 바꾸십시오.

## Hand retargetting 프로젝트

`ros2_ws/src/hand_retargetting/packages/`에 Hand용 ROS 2 패키지 소스가 있습니다.
MANUS SDK, WUJI 드라이버, 독립 Docker 설정도 같은 submodule에 있습니다. 기본
`ros2_ws` 빌드와 Docker 이미지는 Hand 패키지를 포함하지 않습니다. Hand 루트의
`COLCON_IGNORE`가 기본 colcon 탐색을 막고, `.dockerignore`는 Hand 저장소를 기본 Docker
context에서 제외합니다. `INCLUDE_HAND=1`일 때는 Hand 저장소 자체의 Docker 설정으로
별도 이미지를 빌드합니다. Hand를 workspace에 추가 빌드할 때는 `packages/`를 colcon의
`--base-paths`로 명시하고 하드웨어 의존성을 준비합니다.
현재 Hand 경로는 MANUS 장갑 입력을 WUJI 손 관절 명령으로 변환합니다.

저장소를 일반 clone으로 받았다면 submodule을 초기화합니다. SDK 바이너리는 Git LFS로
관리되므로 손 리타게팅을 사용할 때만 Git LFS를 설치하고 해당 저장소에서 가져오면 됩니다.

```bash
git submodule update --init --recursive ros2_ws/src/hand_retargetting
sudo apt install git-lfs
git lfs install
git -C ros2_ws/src/hand_retargetting lfs pull
```

LFS SDK가 실제 바이너리로 내려받아져 있어야 Hand 이미지를 빌드할 수 있습니다. Hand만
빌드하려면 `make docker-build-hand`를 사용하고, MPC/DCP3까지 모두 새로 빌드하려면
`make docker-build INCLUDE_HAND=1`을 사용합니다. 실행 순서와 WUJI `serial_number` 지정은
[Make 명령과 실행 흐름](#make-명령과-실행-흐름)을 참고하세요. Hand 이미지에는 MANUS SDK가
포함되므로 공개 레지스트리에 업로드하지 말고 라이선스 범위 안에서 관리하세요.

## 모듈 경계

`ros2_ws/src/indy7_mpc`는 프레임워크가 소유합니다. Vive는 독립 Git 저장소라
`ros2_ws/src/vive_tracker_ros2`에 바로 clone됩니다. Hand는 현재 MANUS 메시지·입력,
MANUS-WUJI 리타게팅과 WUJI submodule을 한 Git 저장소가 관리하므로
`ros2_ws/src/hand_retargetting`에 한 번만 clone합니다. Git은 저장소의 하위 폴더만
독립적으로 clone할 수 없습니다. 향후 MANUS와 retargeting의 독립 원격 저장소가
생기면 각각을 `ros2_ws/src/`에 병렬로 clone하고 이 묶음 submodule을 제거하면 됩니다.

WUJI 손은 로봇의 끝단 장치이지만 Hand 저장소 전체가 Indy7 전용은 아닙니다. MANUS는
조작자 입력, retargeting은 입력에서 손 관절 명령으로의 변환, WUJI 패키지는 끝단 장치
드라이버를 담당합니다. 실제 Indy7+WUJI 조합의 URDF·launch·설정이 생기면 그 조합은
`robots/indy7/`에서 관리하고, ROS 노드로 구현하는 연결 코드는 `ros2_ws/src/`에 둡니다.

현재 Hand 프로젝트 내부에서는 MANUS 입력에서 WUJI 관절 명령까지 연결되어 있습니다.
Vive Tracker는 `/{namespace}/{tracker_id}/pose`를 발행하고 Indy7 MPC는 기본적으로
`teleop/target_pose`를 구독하므로, Tracker로 팔을 제어하려면 좌표계 변환·캘리브레이션을
담당하는 별도 ROS 2 adapter가 필요합니다. 그 adapter는 이 프레임워크의
`ros2_ws/src/`에 두는 것이 적절합니다.

## Verify the image

```bash
make docker-test
```

또는 컨테이너 안에서 테스트 결과를 직접 확인합니다.

```bash
docker run --rm robot-teleoperation-framework:mpc \
  python3 -m colcon test-result --verbose
```

ROS와 분리된 MPC 드라이런도 이미지 안에서 실행합니다.

```bash
docker run --rm robot-teleoperation-framework:mpc \
  python3 indy7_mpc_test.py

docker run --rm robot-teleoperation-framework:mpc \
  python3 indy7_mpc_test.py \
  --target '0.1,-0.2,0.3,0,0.2,-0.1'
```

## Run the simulation

컨테이너 셸을 열려면 다음을 실행합니다. 저장소 전체가 컨테이너의 `/workspace`에
양방향 bind mount되고, 컨테이너 시작 시 해당 로컬 소스로 ROS 패키지를
`--symlink-install` 빌드합니다. 호스트에서 파일을 수정하면 컨테이너에서도 바로 보이며,
컨테이너에서 만든 파일도 호스트에 반영됩니다. 생성 파일은 호스트 UID/GID로 기록됩니다.

MPC 시뮬레이션은 자동 시작하지 않습니다.

```bash
make docker-run
```

셸 안에서 필요한 실행 명령을 직접 입력할 수 있습니다. 예를 들어 headless 시뮬레이션은:

```bash
ros2 launch indy7_mpc indy7_mpc.launch.py rviz:=false plot:=false
```

Python 소스 수정은 symlink된 파일에 반영됩니다. 이미 실행 중인 노드는 변경된 코드를
자동 reload하지 않으므로 다시 실행하세요. `setup.py`, `package.xml` 같은 패키지 설정을
바꿨다면 셸 안에서 `cd /workspace/ros2_ws && colcon build --symlink-install --merge-install`
을 다시 실행합니다.

Linux X11 데스크톱에서 RViz를 호스트 화면에 띄우려면 다음을 실행합니다. 대상은 호스트의
`DISPLAY`와 Xauthority 쿠키를 컨테이너에 전달합니다.

```bash
make docker-run-gui
```

이 명령은 `XAUTHORITY` 환경 변수를 사용하며, 설정되지 않았으면 `~/.Xauthority`를
사용합니다. 컨테이너에는 Xauthority 파일을 읽기 전용으로 연결합니다.

GUI 컨테이너는 기본적으로 소프트웨어 OpenGL 렌더링을 사용합니다. GPU 장치를
전달하지 않는 환경에서 `failed to load driver: iris` 오류로 RViz가 표시되지 않는
문제를 방지합니다. 기존 컨테이너에는 설정이 자동 반영되지 않으므로, 실행 중인
launch를 종료하고 `make docker-up-indy7-sim`으로 컨테이너를 갱신한 뒤
`make docker-launch-indy7-sim`을 다시 실행하세요. `make docker-run-gui`로 연 셸은
종료 후 같은 명령으로 다시 여세요. 이미지 재빌드는 필요 없습니다.

컨테이너를 직접 실행할 때는 `bash`를 명령으로 지정해 셸을 열 수 있습니다.
`--net=host`와 `--ipc=host`는 ROS 2 DDS 통신을 호스트와 맞추기 위한 옵션입니다.

```bash
docker run --rm -it --net=host --ipc=host \
  --user "$(id -u):$(id -g)" -e HOME=/tmp \
  -v "$PWD:/workspace" -w /workspace \
  robot-teleoperation-framework:mpc bash
```

직접 실행한 셸에서 로컬 소스로 빌드하려면:

```bash
cd /workspace/ros2_ws
colcon build --symlink-install --merge-install
source install/setup.bash
```

이미지 기본 launch 명령을 직접 실행하려면 다음과 같이 명시합니다.

```bash
docker run --rm --net=host --ipc=host \
  robot-teleoperation-framework:mpc \
  ros2 launch indy7_mpc indy7_mpc.launch.py rviz:=false plot:=false
```

컨테이너를 백그라운드에서 유지하면서 토픽을 확인하려면 이름을 지정합니다.

```bash
docker run -d --name robot-teleoperation --net=host --ipc=host \
  robot-teleoperation-framework:mpc

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

- [Indy7 MPC 시작점](robots/indy7/README.md)
- [ROS 2 패키지, 토픽, 파라미터 및 실험 방법](ros2_ws/README.md)
- [MANUS-WUJI 손 리타게팅 모듈](ros2_ws/src/hand_retargetting/README.md)

## Hardware boundary

`indy7_dcp3`는 Neuromeka의 `IndyDCP3` client를 통해 `/indy7_dcp3/joint_states`를 내보내고,
`/indy7_dcp3/joint_position_cmd`를 `TELE_JOINT_ABSOLUTE` 관절 명령으로 전달합니다. 기본은
disarmed이며 명령과 deadman이 계속 들어오고 로봇이 IDLE일 때만 arm할 수 있습니다.
deadman 또는 MPC 명령이 끊기거나 관절 제한을 벗어나면 teleop을 정지합니다.

실기 MPC의 초기 목표는 첫 실제 joint state에서 계산한 현재 TCP pose입니다. 예제 YAML의
목표 위치를 자동 실행하지 않습니다. `docker-up-indy7`은 GUI/DCP3 개발 컨테이너만 idle로
준비하며 ROS 노드는 자동 launch하지 않습니다. 각 컨테이너 shell에 들어가 DCP3와 MPC
launch를 직접 실행하십시오.

```bash
make docker-up-indy7 ROBOT_IP=controller-address
```

`controller-address`를 실제 Indy7 제어기 IP 주소로 바꿉니다.

이 명령 자체는 로봇에 연결하거나 arm하지 않습니다. 피드백과 MPC 목표가 정상인지 별도로
확인한 다음, DCP3 shell에서 deadman을 유지하고 명시적으로 arm해야 합니다. 상세 절차는
[`ros2_ws/src/indy7_dcp3/README.md`](ros2_ws/src/indy7_dcp3/README.md)를 참고하세요.

실기 실행 순서는 DCP3 adapter, 실기 MPC/RViz, keyboard, 지속 deadman publisher 순입니다.
deadman publisher는 실행 중인 채로 두고, 별도 DCP3 shell에서만 arm/disarm 서비스를
호출합니다. 종료할 때는 먼저 `data: false`로 명시적으로 disarm한 뒤 launch와 publisher를
종료합니다. 각 shell은 ROS와 마운트된 workspace overlay를 자동 source하므로 매번 수동
`source`나 `colcon build`를 반복하지 않습니다. 소스는 `/workspace`에 read-only bind mount되어
있어 편집은 호스트에서 하고, Python/launch/YAML 변경은 관련 노드를 재시작하면 반영됩니다.

MPC, ros2_control 시뮬레이션, DCP3 피드백/명령 주기 설정은 모두 200 Hz(5 ms)입니다.
시뮬레이션은 `calculate_dynamics: true`로 위치 변화에 따른 관절 속도를 계산합니다.
MPC YAML의 `cartesian_max_speed`는 `0.5 m/s`, `cartesian_max_angular_speed`는
`3.0 rad/s`입니다. DCP3의 `max_joint_velocity` 파라미터와 주기별 목표 커팅은
제거했으며, MPC의 관절 속도·증분 제한과 DCP3 속도·가속도 비율 20%는 적용됩니다.
먼저 시뮬레이션에서 확인하고,
실기에서는 `cartesian_max_speed`만 보수적으로 한 단계씩 바꿔 작은 직선 동작으로 검증합니다.
계획 상한과 실제 TCP 응답을 구분해 진단 로그에서 확인합니다.
YAML 수정은 호스트 소스에 바로 반영되므로 이미지 재빌드 없이 MPC를 재시작하면 됩니다.

이 adapter는 충돌/작업공간 감시나 안전 정격 E-stop이 아닙니다. 첫 테스트는 낮은 속도,
빈 작업공간, 물리 E-stop을 바로 누를 수 있는 상태에서 진행하고 제조사 안전 절차를
준수해야 합니다.

## Release checklist

1. `make docker-build`가 새 환경에서 성공하는지 확인합니다.
2. `make docker-test`와 headless 시뮬레이션이 통과하는지 확인합니다.
3. 실제 장비 연결 전 adapter limit/watchdog/deadman을 비실기 테스트하고 물리 E-stop을 확인합니다.
4. 공개 배포 전 저장소의 최상위 라이선스를 프로젝트 정책에 맞게 결정합니다.
