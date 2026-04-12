# rtc_tsid

> 이 패키지는 [RTC Framework](../README.md) 워크스페이스의 일부입니다.

## 개요

RTC 프레임워크의 **TSID (Task-Space Inverse Dynamics) QP 솔버 라이브러리**입니다. Weighted QP (WQP)와 Hierarchical QP (HQP) 두 가지 formulation을 지원하며, ProxSuite 백엔드를 사용한 실시간 안전한 전신 제어(Whole-Body Control)를 제공합니다.

**설계 원칙:**
- Strategy 패턴으로 WQP/HQP formulation 교체 가능
- 사전 할당(pre-allocation)을 통한 RT-safe compute 경로
- YAML 기반 phase preset 설정
- Pinocchio 공유 캐시로 중복 동역학 계산 방지

---

## 패키지 구조

```
rtc_tsid/
├── include/rtc_tsid/
│   ├── core/
│   │   ├── controller_base.hpp         -- TSID 내부 컨트롤러 추상 인터페이스
│   │   ├── task_base.hpp               -- 태스크 추상 인터페이스 (J, r, weight, priority)
│   │   ├── constraint_base.hpp         -- 제약조건 추상 인터페이스 (등식/부등식)
│   │   └── formulation_base.hpp        -- Formulation 전략 인터페이스
│   ├── controller/
│   │   └── tsid_controller.hpp         -- TSIDController (final 구현체)
│   ├── formulation/
│   │   ├── wqp_formulation.hpp         -- Weighted QP formulation
│   │   ├── hqp_formulation.hpp         -- Hierarchical QP formulation (cascaded)
│   │   └── formulation_factory.hpp     -- Formulation 팩토리
│   ├── tasks/
│   │   └── posture_task.hpp            -- 관절 자세 추종 태스크
│   ├── constraints/
│   │   ├── eom_constraint.hpp          -- 운동 방정식 등식 제약
│   │   ├── contact_constraint.hpp      -- 접촉 제약
│   │   ├── friction_cone_constraint.hpp -- 마찰 원뿔 부등식 제약
│   │   └── torque_limit_constraint.hpp -- 토크 한계 부등식 제약
│   ├── types/
│   │   ├── wbc_types.hpp               -- 핵심 데이터 구조체
│   │   └── qp_types.hpp                -- QP 문제 구조체
│   └── solver/
│       └── qp_solver_wrapper.hpp       -- ProxSuite QP 솔버 래퍼
├── src/                                -- 구현 파일
├── config/                             -- YAML 설정 파일
├── test/                               -- 11개 GTest 파일
├── CMakeLists.txt
└── package.xml
```

---

## 핵심 컴포넌트

### TSIDController (`controller/tsid_controller.hpp`)

TSID 솔버의 메인 컨트롤러입니다. `ControllerBase`를 상속하며 `final`로 선언되어 있습니다.

> **참고:** `RTControllerInterface`(ROS2 제어 인터페이스)와는 별도의 내부 인터페이스입니다. ROS2 통합은 향후 Phase 3에서 진행 예정입니다.

| 메서드 | 설명 |
|--------|------|
| `init(model, robot_info, config)` | Pinocchio 모델 + YAML 설정으로 초기화 |
| `compute(state, ref, cache, contacts)` | QP 솔버 실행, `CommandOutput` 반환 (`noexcept`) |
| `reset()` | 내부 상태 초기화 (`noexcept`) |
| `name()` | `"tsid"` 반환 (`noexcept`) |
| `apply_phase_preset(name)` | YAML에서 정의된 phase preset 적용 (`noexcept`) |
| `activate_contact(idx)` | 접촉점 활성화 (`noexcept`) |
| `deactivate_contact(idx)` | 접촉점 비활성화 (`noexcept`) |
| `formulation()` | 내부 formulation 객체 참조 반환 |

---

### Formulation (WQP / HQP)

| Formulation | 설명 | solve_levels |
|-------------|------|-------------|
| **WQP** | 모든 태스크를 가중치로 통합하여 단일 QP로 해결 | 1 |
| **HQP** | 우선순위별 cascaded QP — 상위 레벨 null-space에서 하위 레벨 최적화 | N (레벨 수) |

`FormulationFactory`를 통해 YAML `formulation_type` 값으로 생성:
```yaml
formulation_type: "wqp"  # 또는 "hqp"
```

---

### 태스크 (Tasks)

| 태스크 | 설명 |
|--------|------|
| `PostureTask` | 관절 자세 추종: `r = q_ref - q`, `J = I` (joint space identity) |

태스크는 `TaskBase`를 상속하며, `compute_residual()` 메서드로 잔차 벡터와 자코비안을 반환합니다. 각 태스크는 `weight`와 `priority` 속성을 가집니다.

---

### 제약조건 (Constraints)

| 제약조건 | 타입 | 설명 |
|---------|------|------|
| `EOMConstraint` | 등식 | 운동 방정식: @f$ M \ddot{q} + h = S^T \tau + J_c^T \lambda @f$ |
| `ContactConstraint` | 등식 | 접촉점 가속도 = 0: @f$ J_c \ddot{q} + \dot{J}_c \dot{q} = 0 @f$ |
| `FrictionConeConstraint` | 부등식 | 쿨롱 마찰 원뿔 선형 근사 |
| `TorqueLimitConstraint` | 부등식 | 액추에이터 토크 상/하한 |

---

## 핵심 데이터 타입 (`types/wbc_types.hpp`)

### RobotModelInfo

| 필드 | 타입 | 설명 |
|------|------|------|
| `nq`, `nv` | `int` | 설정/속도 공간 차원 |
| `n_actuated` | `int` | 구동 관절 수 |
| `floating_base` | `bool` | 부유 베이스 여부 |
| `S` | `MatrixXd` | 선택 행렬 [n_actuated × nv] |
| `tau_max`, `tau_min` | `VectorXd` | 토크 상/하한 |
| `q_upper`, `q_lower` | `VectorXd` | 관절 위치 상/하한 |
| `v_max` | `VectorXd` | 최대 관절 속도 |

### PinocchioCache

공유 동역학 계산 캐시. `update()` 호출 시 질량 행렬, 비선형항, 중력, 접촉 자코비안 등을 일괄 갱신합니다.

| 필드 | 타입 | 설명 |
|------|------|------|
| `M` | `MatrixXd` | 질량 행렬 [nv × nv] |
| `h`, `g` | `VectorXd` | 비선형항, 중력항 |
| `q`, `v` | `VectorXd` | 현재 설정/속도 |
| `contact_frames` | `vector<FrameCache>` | 접촉 프레임별 자코비안 [6 × nv] + dJ·v |

### CommandOutput

| 필드 | 타입 | 설명 |
|------|------|------|
| `tau` | `VectorXd` | 구동 토크 [n_actuated] |
| `a_opt` | `VectorXd` | 최적 가속도 [nv] |
| `lambda_opt` | `VectorXd` | 접촉력 [active_contact_vars] |
| `qp_converged` | `bool` | QP 솔버 수렴 여부 |
| `solve_time_us` | `double` | 풀이 시간 [μs] |
| `solve_levels` | `int` | WQP: 1, HQP: N |

---

## 의존성

| 의존성 | 용도 |
|--------|------|
| `pinocchio` | 강체 동역학 (질량 행렬, 자코비안, RNEA 등) |
| `Eigen3` | 선형 대수 (행렬/벡터 연산) |
| `proxsuite` | Dense QP 솔버 (사전 할당 워크스페이스) |
| `yaml-cpp` | YAML 설정 파싱 |
| `ament_cmake` | 빌드 시스템 |
| `ament_cmake_gtest` | 테스트 (test_depend) |

---

## 빌드

```bash
colcon build --packages-select rtc_tsid
```

---

## 테스트

```bash
colcon test --packages-select rtc_tsid --event-handlers console_direct+
colcon test-result --verbose
```

11개 테스트:

| 테스트 | 설명 |
|--------|------|
| `test_qp_solver_wrapper` | ProxSuite QP 래퍼 기본 동작 |
| `test_wbc_types` | WBC 타입 시스템 초기화/갱신 |
| `test_posture_task` | 자세 태스크 잔차/자코비안 |
| `test_eom_constraint` | 운동 방정식 등식 제약 |
| `test_contact_constraint` | 접촉 등식 제약 |
| `test_friction_cone_constraint` | 마찰 원뿔 부등식 제약 |
| `test_torque_limit_constraint` | 토크 한계 부등식 제약 |
| `test_tsid_wqp` | WQP formulation 통합 테스트 |
| `test_tsid_hqp` | HQP formulation 통합 테스트 |
| `test_tsid_wqp_hqp_compare` | WQP vs HQP 비교 검증 |
| `test_tsid_performance` | 성능 벤치마크 |

---

## 라이선스

MIT License
