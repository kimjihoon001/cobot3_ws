# smartfarm_common

초기 설계에서 공통 로깅·트레이 관리 기능을 둘 예정이었던 ROS 2 Python
패키지다. 현재 실행 노드와 공용 모듈은 없으며 패키지 골격만 유지한다.

현행 공통 기능의 위치는 다음과 같다.

- 메시지·서비스: `smartfarm_interfaces`
- 통합 launch: `smartfarm_bringup`
- MM/IW/Forklift 상태 연결: 각 기능 패키지의 실제 미션 노드

새 공통 기능이 생기기 전까지 이 패키지를 launch하거나 `ros2 run`으로 실행할
대상은 없다. 초기 `logger_node`, `tray_manager_node` 등의 폐기 이력은
[`../INTERFACES.md`](../INTERFACES.md)를 참고한다.
