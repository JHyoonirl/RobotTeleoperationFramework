# Indy7 MPC 시작점

`indy7_mpc_test.py`는 Neuromeka Indy7(6-DOF)의 관절 위치 추종을 위한 안전한 MPC 드라이런이다. ROS2 MPC는 Crocoddyl의 첫 예측 관절 위치를 `q_cmd`로 발행하고, 시뮬레이션에서는 `forward_position_controller`, 실기에서는 IndyDCP3 teleoperation adapter로 연결한다.

```bash
cd /path/to/RobotTeleoperationFramework
.venv/bin/python manipulator/indy7_mpc_test.py
```

목표는 라디안 6개로 지정한다.

```bash
.venv/bin/python manipulator/indy7_mpc_test.py --target '0.1,-0.2,0.3,0,0.2,-0.1'
```

실장 전에는 Indy7 드라이버에서 매 주기 `position`, `velocity`를 읽고 `q_cmd`를 IndyDCP3 `TELE_JOINT_ABSOLUTE / movetelej_abs()`로 전달해야 한다. 변환에는 실제 조인트 제한, 통신 watchdog, 충돌 감시, 비상정지, deadman 스위치가 필요하다.

ROS2 + Crocoddyl 패키지는 [`ros2_ws/README.md`](ros2_ws/README.md)를 참고한다.
