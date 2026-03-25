# 변경 이력 — rtc_msgs

이 문서는 [Keep a Changelog](https://keepachangelog.com/ko/1.1.0/) 형식을 따르며,
[Semantic Versioning](https://semver.org/lang/ko/)을 준수합니다.

## [5.17.0] - 2026-03-25

### 변경 (Changed)

- 워크스페이스 전체 버전 v5.17.0 통일
- 문서 최신화 (README.md, package.xml)

---

## [5.14.0] - 2026-03-14

### 추가 (Added)

- **초기 생성**: RT Controller 스택을 위한 커스텀 ROS2 메시지 정의 패키지 (ur5e_msgs에서 rtc_msgs로 이름 변경)
- `JointCommand.msg` — 로봇 암 관절 명령 (position/torque), joint_names 기반 매핑
- `HandCommand.msg` — 핸드 모터 명령, motor_names 기반 매핑
- `HandMotorState.msg` — 핸드 모터 피드백 (위치/속도)
- `FingertipSensor.msg` — 단일 핑거팁 센서 (기압 8개 + ToF 3개)
- `HandSensorState.msg` — 전체 핸드 센서 상태 (FingertipSensor 배열)
- `rosidl_default_generators` 기반 자동 C++/Python 바인딩 생성
- `std_msgs/Header` 타임스탬프 지원
