# Indy7 MPC 시작점

이 디렉터리의 실행·검증은 Docker 컨테이너 안에서 수행합니다. 호스트에서 Python
virtualenv를 만들거나 ROS 2를 source할 필요가 없습니다.

저장소 루트에서 이미지를 먼저 빌드합니다.

```bash
cd /path/to/RobotTeleoperationFramework
make docker-build
```

`indy7_mpc_test.py`는 Neuromeka Indy7(6-DOF)의 관절 위치 추종을 위한 ROS 2와 분리된
최소 MPC 드라이런입니다.

```bash
docker run --rm robot-teleoperation-framework \
  python3 indy7_mpc_test.py
```

목표는 라디안 6개로 지정합니다.

```bash
docker run --rm robot-teleoperation-framework \
  python3 indy7_mpc_test.py \
  --target '0.1,-0.2,0.3,0,0.2,-0.1'
```

ROS 2 패키지의 mock hardware 시뮬레이션은 다음처럼 실행합니다.

```bash
docker run --rm --net=host --ipc=host \
  robot-teleoperation-framework
```

상세한 launch 옵션, 토픽, YAML 파라미터, 로그 분석, 실기 adapter 경계는
[`ros2_ws/README.md`](ros2_ws/README.md)를 참고합니다.

실장 전에는 Indy7 드라이버에서 매 주기 `position`, `velocity`를 읽고
`mpc/joint_position_cmd`를 IndyDCP3 `TELE_JOINT_ABSOLUTE / movetelej_abs()`로 전달해야
합니다. 변환에는 실제 joint limit, 통신 watchdog, 충돌 감시, 비상정지, deadman 스위치가
필요합니다.
