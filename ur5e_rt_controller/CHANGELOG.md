# 변경 이력 — ur5e_rt_controller

이 파일은 [Keep a Changelog](https://keepachangelog.com/ko/1.0.0/) 형식을 따르며,
[시맨틱 버전 관리](https://semver.org/lang/ko/)를 사용합니다.

---

## [5.7.0] - 2026-03-11

### 수정 (Fixed) — PController 정상상태 오차 제거

- **원인**: `command = kp * error`를 절대 위치 명령으로 전송 → MuJoCo 서보가 `ctrl=kp*error` 위치로 구동
  - 평형: `q* = kp/(kp+1) * target` (예: kp=1 → target 90° 시 45°에 정착)
- **수정**: 증분 위치 스텝 방식으로 변경
  ```cpp
  output.robot_commands[i] = state.robot.positions[i]
                            + gains_.kp[i] * error * state.robot.dt;
  ```
- **평형**: `q* = target` (정상상태 오차 없음)

---

## [5.6.1] - 2026-03-10

### 변경 (Changed) — 궤적 생성 개선

- `PinocchioController`에 `JointSpaceTrajectory<6>` 추가 (직접 점프 → 5차 관절공간 궤적, `trajectory_speed` 파라미터)
- `OperationalSpaceController`에 `TaskSpaceTrajectory` 추가 (SE(3) 5차 스플라인, `trajectory_speed` + `trajectory_angular_speed`)
- `ClikController`에서 미사용 `trajectory_angular_speed` 제거 (dead-code)

---

## [5.5.0] - 2026-03-09

### 변경 (Changed) — 소스 분할 및 클래스명 변경

- `rt_controller.cpp` → `rt_controller_node.hpp` + `rt_controller_node.cpp` + `rt_controller_main.cpp` 3분할
- 클래스명 변경: `CustomController` → `RtControllerNode`
- `SpscLogBuffer` 비트 AND 연산 최적화

---

## [5.4.0] - 2026-03-09

### 추가 (Added) — Controller Registry 패턴

- `MakeControllerEntries()` 팩토리 리스트: 새 컨트롤러 등록 시 한 줄만 추가
- `RTControllerInterface`에 `LoadConfig()` / `UpdateGainsFromMsg()` 훅 추가
- 컨트롤러별 YAML 로딩·게인 업데이트를 각 컨트롤러 자신이 담당
- 기존 `switch`/`dynamic_cast` 코드 제거

---

## [5.3.0] - 2026-03-08

### 추가 (Added) — 런타임 컨트롤러 전환

- P/PD/Pinocchio/CLIK/OSC 컨트롤러를 `/rt_controller/controller_type` 토픽으로 런타임 전환
- `/rt_controller/controller_gains` 토픽으로 동적 게인 업데이트
- 5차 다항식 궤적 생성 서브시스템 (`trajectory/`): `QuinticPolynomial`, `TaskSpaceTrajectory`, `JointSpaceTrajectory<N>`

---

## [5.0.0] - 2026-03-07

### 추가

- **초기 분리**: 단일 패키지에서 500Hz RT 제어기를 독립 패키지로 추출
- SCHED_FIFO 멀티스레드 아키텍처 (4 executor, CPU 코어 할당)
- 전략 패턴 기반 컨트롤러: `PDController`, `PController`, `PinocchioController`, `ClikController`, `OperationalSpaceController`
- E-STOP 안전 시스템 (50Hz 워치독)
- 잠금-없는 `SpscLogBuffer` + `DataLogger` CSV 로깅
- `ControllerTimingProfiler` 잠금-없는 타이밍 프로파일러
