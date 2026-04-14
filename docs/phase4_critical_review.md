# Phase 4: DemoWbcController — 비판적 분석 및 구현 계획

> **작성일**: 2026-04-14
> **입력**: `phase4_demo_wbc_prompt_v2.md`
> **상태**: 코드베이스 검증 완료, 구현 전 최종 계획

---

## 1. 프롬프트 vs 실제 코드베이스 불일치

### 1.1 Critical — 시그니처 불일치 (반드시 수정)

| # | 프롬프트 | 실제 (`rt_controller_interface.hpp`) | 영향 |
|---|---------|--------------------------------------|------|
| C1 | `SetDeviceTarget(const ControllerState&, const DeviceState&)` | `SetDeviceTarget(int device_idx, std::span<const double> target)` | 컴파일 불가 |
| C2 | `UpdateGainsFromMsg(const std::vector<double>&)` | `UpdateGainsFromMsg(std::span<const double> gains)` | 컴파일 불가 |
| C3 | `output.devices[0].positions[i]` | `output.devices[0].commands[i]` (주 출력) + `target_positions`, `trajectory_positions`, `goal_positions` 등 별도 필드 | 출력 누락 |
| C4 | `tsid_controller_.compute(cache, ref, contacts)` 3인자 | `compute(state, ref, cache, contacts)` 4인자 — `ControlState` 추가 | 컴파일 불가 |
| C5 | `result.success` / `result.acceleration` | `result.qp_converged` / `result.a_opt` | 컴파일 불가 |
| C6 | `~DemoWbcController() = default` + 기본 생성자 public | base는 복사/이동 delete, 소멸자 non-virtual-default, 생성자 protected | 컴파일 불가 |

### 1.2 Warning — API 이름/네임스페이스 불일치

| # | 프롬프트 | 실제 |
|---|---------|------|
| W1 | `apply_preset()` | `apply_phase_preset()` |
| W2 | namespace `tsid::` | `rtc::tsid::` |
| W3 | `contact_state_.contacts` 직접 순회 | `ContactState::for_each_active()` 사용 권장 |

### 1.3 Missing — 프롬프트에 빠진 필수 패턴

| # | 누락 항목 | 기존 패턴 (DemoJointController) | 필요성 |
|---|----------|-------------------------------|--------|
| M1 | E-STOP 인터페이스 | `TriggerEstop()`, `ClearEstop()`, `IsEstopped()`, `SetHandEstop()` override | E-STOP 미처리 시 안전 위험 |
| M2 | `GetCommandType()` / `command_type_` | YAML `command_type` 파싱 + getter | controller manager가 호출 |
| M3 | `GetCurrentGains()` | GUI "Load Gain" 기능 | GUI 연동 |
| M4 | `OnDeviceConfigsSet()` | device joint limits, frame ID resolve | joint limit, FK frame 설정 |
| M5 | `OnSystemModelConfigSet()` | hook이 `LoadConfig()` 전에 불림 — 아님, `LoadConfig()` 내에서 `GetSystemModelConfig()` 확인 후 모델 구축 | 모델 초기화 타이밍 |
| M6 | `num_channels` / `num_devices` 출력 설정 | `output.num_devices = state.num_devices`, `out.num_channels = nc` | 미설정 시 publish 안됨 |
| M7 | `goal_positions`, `target_positions`, `trajectory_positions/velocities` | WriteOutput에서 모든 필드 설정 | GUI, logging, digital twin에 필요 |
| M8 | Logging (`rclcpp::Logger` + throttle) | `DemoJointLogger()`, `RCLCPP_INFO_THROTTLE` | 디버깅 필수 |
| M9 | Mutex + atomic target 보호 | `target_mutex_` + `robot_new_target_` atomic | 스레드 안전 |
| M10 | `JointSpaceTrajectory<N>` 재사용 | quintic 직접 구현 대신 기존 라이브러리 | 코드 중복 방지 |
| M11 | `ClampCommands()` 패턴 | `device_position_lower_/upper_` 기반 clamp | joint limit 보호 |
| M12 | Sensor data 읽기 (ReadState) | fingertip sensor, inference data 파싱 | contact 감지 |
| M13 | `DemoSharedConfig` 활용 | 공통 설정 (grasp params, vtcp 등) | 설정 일관성 |

---

## 2. 아키텍처 결정 사항

### 2.1 Combined Model 전략 (사용자 결정: combined model 사용)

**결정**: `PinocchioModelBuilder::GetFullModel()`로 16-DOF combined 모델 사용.

**구현 방식**:
- `rtc_urdf_bridge`가 xacro 처리 + full model 빌드를 모두 지원 (검증 완료)
- 시스템 YAML (`ur5e_sim.yaml`)의 `urdf:` 섹션이 이미 combined xacro 경로를 가짐
- `GetFullModel()` → 16-DOF (arm 6 + hand 10) Pinocchio model
- 별도 flattened URDF 생성 불필요 — `PinocchioModelBuilder`가 xacro를 내부 처리

**모델 구성**:

```
PinocchioModelBuilder(system_model_config)
  ├─ GetFullModel()    → 16 DOF  → TSID용 (PinocchioCache에 전달)
  ├─ GetReducedModel("ur5e") → 6 DOF  → non-TSID phase FK용 (기존 패턴)
  └─ GetTreeModel("hand")   → 10 DOF → hand FK용 (기존 패턴)
```

**Joint Ordering 해결**:
- `RtModelHandle::SetJointOrder()` 사용 (full model에 적용)
- `DeviceNameConfig` joint_state_names (arm 6개 + hand 10개)를 연결하여 16개 joint order 설정
- `ExtractState()`: device[0].positions[0..5] + device[1].positions[0..9] → q[0..15] (reorder 적용)

### 2.2 PinocchioCache vs RtModelHandle 역할 분리

| 용도 | 사용 객체 | 이유 |
|------|----------|------|
| TSID phase (kPreGrasp, kClosure, kHold) | `PinocchioCache` (rtc_tsid) | M, h, g, contact Jacobian 등 TSID QP에 필요한 모든 양을 계산 |
| Non-TSID phase FK (approach distance 등) | `RtModelHandle` (rtc_urdf_bridge) | 경량 FK, 기존 패턴 |
| Output assembly 시 task-space logging | `RtModelHandle` (arm sub-model) | 기존 DemoJoint 패턴 일관성 |

**PinocchioCache 초기화**:
```cpp
pinocchio_cache_.init(builder_->GetFullModel(), contact_manager_config_);
// shared_ptr<const Model> 전달 → lifetime 보장
// contact_manager_config_ → fingertip frame 등록 + Jacobian 버퍼
```

### 2.3 Phase Preset RT-safety

**문제**: `apply_phase_preset(const std::string&)`는 `unordered_map::find` 호출 → hash + string 비교 → RT 경로 위험.

**해결**: `OnPhaseEnter()`에서만 호출 (이미 프롬프트에서 이 패턴). `OnPhaseEnter()`는 phase 전환 시 1회만 호출되므로, occasional string lookup은 수용 가능 (매 tick 아님).

**대안** (더 안전): LoadConfig() 시 preset을 `PhasePreset` struct로 pre-resolve하여 `std::array<PhasePreset, kNumPhases>`에 저장, RT path에서는 struct overload `apply_phase_preset(const PhasePreset&)` 사용.

→ **pre-resolve 방식 채택** (RT-safe 보장).

### 2.4 ForceTask 역할 (Position-only 환경)

프롬프트의 분석이 정확. Position 출력 환경에서 ForceTask는:
- QP 내에서 contact force distribution 최적화에 기여
- 실제 force 실현은 position trajectory + hardware compliance에 의존
- **Phase 4 MVP에서는 ForceTask를 비활성 상태로 시작**, kClosure/kHold에서만 활성화

---

## 3. MVP 범위 결정

### Phase 4A (MVP — 이번 구현)
- **kIdle**: position hold (기존 패턴)
- **kApproach**: quintic trajectory to pre-grasp pose (JointSpaceTrajectory 재사용)
- **kPreGrasp**: TSID → position (contact 없음, SE3Task + PostureTask)
- **kFallback**: position hold (QP 실패 시)
- FSM skeleton: 모든 phase 코드 구조 + 전환 조건
- TSID → position 적분 로직 완전 구현
- 빌드 + 단위 테스트

### Phase 4B (후속 — contact 연동)
- **kClosure**: contact 활성화, ForceTask 활성화
- **kHold**: grasp 유지, anomaly monitoring
- **kRetreat**: 후퇴 trajectory
- **kRelease**: finger open
- Sensor data integration (fingertip force/contact)
- 시뮬레이션 통합 테스트

---

## 4. 업데이트된 구현 계획

```
Goal: DemoWbcController — TSID QP → Position integration controller with FSM
Scope: ur5e_bringup (주)
의존: rtc_tsid (라이브러리), rtc_urdf_bridge (모델 빌드)

Step 1/6 — 헤더 설계 (interface-first)
  Files: ur5e_bringup/include/ur5e_bringup/controllers/demo_wbc_controller.hpp
  Work:
    - RTControllerInterface 정확한 시그니처 준수:
      - Compute(const ControllerState&) noexcept
      - SetDeviceTarget(int device_idx, std::span<const double> target) noexcept
      - InitializeHoldPosition(const ControllerState&) noexcept
      - Name() const noexcept → "DemoWbcController"
      - LoadConfig(const YAML::Node&)
      - UpdateGainsFromMsg(std::span<const double>) noexcept
      - GetCurrentGains() const noexcept
      - GetCommandType() const noexcept
    - E-STOP: TriggerEstop/ClearEstop/IsEstopped/SetHandEstop override
    - OnDeviceConfigsSet() override — joint limits, frame ID resolve
    - WbcPhase enum (8 states)
    - TSID 관련:
      - rtc::tsid::TSIDController tsid_controller_
      - rtc::tsid::PinocchioCache pinocchio_cache_ (full model)
      - rtc::tsid::ContactState contact_state_
      - rtc::tsid::ControlReference control_ref_
      - rtc::tsid::RobotModelInfo robot_info_
      - rtc::tsid::ContactManagerConfig contact_mgr_config_
      - rtc::tsid::CommandOutput tsid_output_
      - std::array<rtc::tsid::PhasePreset, kNumPhases> phase_presets_ (pre-resolved)
    - 모델 관련:
      - PinocchioModelBuilder builder_ (소유)
      - RtModelHandle arm_handle_ (arm sub-model, FK/logging)
      - RtModelHandle full_handle_ (full model, state reorder)
    - 적분 버퍼: q_next_[nv], v_next_[nv], q_min_clamped_[nv], q_max_clamped_[nv], v_limit_[nv]
    - Trajectory: JointSpaceTrajectory<kNumRobotJoints>, JointSpaceTrajectory<kNumHandMotors>
    - Target: target_mutex_, robot_new_target_ atomic, hand_new_target_ atomic
    - GraspTarget struct (pre-grasp/grasp pose, force target)
    - Logging: rclcpp::Logger, rclcpp::Clock
    - E-STOP: atomic<bool> estopped_, hand_estopped_
  Depends: RTControllerInterface, rtc_tsid API (existing)
  Verify: Gate 1 (format), Gate 2 (header-only build check)

Step 2/6 — 핵심 구현 (Compute pipeline + FSM)
  Files: ur5e_bringup/src/controllers/demo_wbc_controller.cpp
  Work:
    A. 초기화:
      - 생성자: urdf_path_ 저장만 (기존 패턴)
      - LoadConfig():
        (1) RTControllerInterface::LoadConfig(cfg) 호출 (topics 파싱)
        (2) GetSystemModelConfig()로 ModelConfig 획득
        (3) PinocchioModelBuilder 생성, arm_handle_ 구축
        (4) full model로 TSID 초기화:
            - RobotModelInfo.build(full_model, tsid_yaml)
            - PinocchioCache.init(full_model_ptr, contact_mgr_config)
            - TSIDController.init(full_model, robot_info, tsid_yaml)
            - ControlReference.init(nq, nv, n_actuated, max_contact_vars)
            - CommandOutput (tsid_output_) pre-allocate
        (5) Phase preset pre-resolve (YAML → PhasePreset struct 배열)
        (6) 적분 버퍼 pre-allocate
        (7) FSM threshold 로드
        (8) command_type_ 파싱
      - OnDeviceConfigsSet():
        (1) joint limits (device_position_lower/upper, device_max_velocity)
        (2) tip_frame_id_ resolve (arm FK logging용)
        (3) full model joint reorder (arm + hand joint names 연결)
    B. Compute() — 500Hz RT:
      ReadState → E-STOP check → UpdatePhase → dispatch:
        - kIdle/kApproach/kRetreat/kRelease → ComputePositionMode()
        - kPreGrasp/kClosure/kHold → ComputeTSIDPosition()
        - kFallback → ComputeFallback()
      → WriteOutput (모든 DeviceOutput 필드 설정)
    C. ComputeTSIDPosition():
      (1) ExtractFullState(): device[0]+device[1] → q_curr_[16], v_curr_[16]
      (2) pinocchio_cache_.update(q_curr_, v_curr_, contact_state_)
      (3) Task reference 갱신 (SE3, Posture, Force)
      (4) ControlState 구성 (q, v, timestamp)
      (5) tsid_output_ = tsid_controller_.compute(ctrl_state, control_ref_, pinocchio_cache_, contact_state_)
      (6) QP 실패 처리: !qp_converged → ++fail_count → fallback
      (7) Semi-implicit Euler: v_next = v_curr + a_opt * dt, q_next = q_curr + v_next * dt
      (8) Velocity/position clamp
    D. ComputePositionMode():
      JointSpaceTrajectory evaluate → position hold if inactive
    E. FSM:
      UpdatePhase() — 전환 조건 체크
      OnPhaseEnter() — preset 적용, trajectory 설정, contact 전환
    F. SetDeviceTarget() — mutex + atomic (기존 패턴)
    G. InitializeHoldPosition() — trajectory 초기화 (기존 패턴)
    H. WriteOutput() — 전체 DeviceOutput 필드 조립 (기존 패턴)
    I. E-STOP handling (기존 패턴)
  Depends: Step 1
  Verify: Gate 1, Gate 2, Gate 4

Step 3/6 — YAML 설정 파일
  Files: ur5e_bringup/config/controllers/demo_wbc_controller.yaml
  Work:
    - topics 섹션: demo_joint_controller.yaml 형식 따름 (role-based)
    - TSID 설정: formulation_type, tasks, constraints, contacts, phase_presets
    - FSM thresholds, integration params
    - gains layout (7 values)
    - command_type: "position"
  Depends: Step 1 (gains layout)
  Verify: YAML 문법

Step 4/6 — 빌드 시스템 통합
  Files:
    - ur5e_bringup/src/controllers/controller_registration.cpp (+3 lines)
    - ur5e_bringup/CMakeLists.txt (+2 lines: source, rtc_tsid dep)
    - ur5e_bringup/package.xml (+1 line: rtc_tsid depend)
  Work:
    - RTC_REGISTER_CONTROLLER(demo_wbc_controller, ...) 추가
    - CMakeLists.txt source list에 demo_wbc_controller.cpp 추가
    - ament_target_dependencies에 rtc_tsid 추가
    - ament_export_dependencies에 rtc_tsid 추가
    - package.xml에 <depend>rtc_tsid</depend> 추가
  Depends: Step 2
  Verify: Gate 2 (full build: ./build.sh -p ur5e_bringup)

Step 5/6 — 단위 테스트
  Files:
    - ur5e_bringup/test/test_demo_wbc_controller.cpp
    - ur5e_bringup/CMakeLists.txt (test 추가)
  Work:
    - FSM 전환: phase 상태 전이 정확성
    - TSID → Position 적분: a=0 → q 유지, 일정 a → 정확한 적분, clamp 동작
    - Quintic trajectory: JointSpaceTrajectory 기반 보간 정확성
    - Fallback: QP N회 실패 → kFallback 전환
    - Output 조립: commands, target_positions, trajectory_positions 정합성
    - E-STOP: trigger → hold, clear → resume
  Depends: Step 4
  Verify: Gate 3

Step 6/6 — 문서 업데이트
  Files:
    - ur5e_bringup/README.md
    - CLAUDE.md (Controllers 테이블, Gains Layout, Key File Locations, Test 테이블)
  Work:
    - DemoWbcController API, FSM, gains layout 문서화
    - TSID integration 아키텍처 설명
    - 테스트 수 업데이트
  Depends: Step 5
  Verify: Gate 5
```

---

## 5. rtc_urdf_bridge 활용 요약

`rtc_urdf_bridge`는 프롬프트의 모든 모델 관련 요구사항을 해결:

| 요구사항 | rtc_urdf_bridge 기능 | 사용법 |
|---------|---------------------|--------|
| Combined URDF (arm+hand) | xacro 자동 처리 | `PinocchioModelBuilder(system_model_config)` — config에 이미 xacro 경로 포함 |
| Full 16-DOF model (TSID용) | `GetFullModel()` | `builder_->GetFullModel()` → `shared_ptr<const pinocchio::Model>` |
| Arm sub-model (FK/logging용) | `GetReducedModel()` | `builder_->GetReducedModel("ur5e")` → 6 DOF |
| Hand tree-model (FK용) | `GetTreeModel()` | `builder_->GetTreeModel("hand")` → 10 DOF |
| Joint order mapping | `RtModelHandle::SetJointOrder()` | arm + hand joint names 연결 → full model reorder |
| Joint limits | `ModelConfig` + `pinocchio::Model` | `model.lowerPositionLimit`, `model.upperPositionLimit`, `model.velocityLimit` |
| Frame ID resolve | `RtModelHandle::GetFrameId()` | fingertip frame, TCP frame |
| RT-safe FK | `RtModelHandle::ComputeForwardKinematics()` | 모든 buffer pre-allocated |
| RT-safe dynamics | `PinocchioCache::update()` (for TSID) | M, h, g, contact Jacobians |

**핵심**: 프롬프트에서 `pinocchio::Model model_`과 `pinocchio::Data data_`를 직접 소유하는 설계는 **불필요**. `RtModelHandle`이 model(shared_ptr) + data(소유)를 캡슐화하고, `PinocchioCache`가 TSID용 계산을 담당.

---

## 6. 프롬프트 원본에서 유지할 사항

다음은 코드베이스와 일치하며 유효한 설계:

- ✅ Position-only output 전략 (TSID acceleration → 이중 적분 → q_next)
- ✅ 8-phase FSM 구조 (단, MVP는 4 phase)
- ✅ 세 가지 제어 모드 (Position Mode / TSID Mode / Fallback)
- ✅ Semi-implicit Euler 적분 방식
- ✅ Safety clamp (position + velocity)
- ✅ QP 연속 실패 → Fallback 전환
- ✅ Phase preset 시스템 (pre-resolve 개선 적용)
- ✅ Contact 활성/비활성은 bool flip만 (RT-safe)
- ✅ 매 tick 센서 기반 closed-loop (open-loop drift 없음)
- ✅ 파일 배치 (`ur5e_bringup/` 내 robot-specific controller)
- ✅ TSID YAML 설정 구조 (tasks, constraints, contacts, phase_presets)
- ✅ Gains layout (7 values: grasp_cmd, force, speeds, weights)
- ✅ 확장성 고려 (MPC interpolation, torque 확장, BT 연동, RL residual)

---

## 7. 위험 요소 및 완화 전략

| 위험 | 심각도 | 완화 |
|------|-------|------|
| TSID QP solve time > 2ms (500Hz budget) | High | 16-DOF QP는 ~200μs 예상 (ProxSuite). Fallback 보호 |
| Full model joint ordering vs DeviceState 불일치 | Medium | SetJointOrder()로 해결. 단위 테스트에서 검증 |
| PinocchioCache.update()의 contact Jacobian overhead | Low | contact 비활성 시 skip. Phase별 contact 수 제한 |
| Phase 전환 시 position 불연속 | Medium | OnPhaseEnter()에서 q_next_ = q_curr_ 설정 (연속성) |
| TSID 적분 velocity drift | Low | 매 tick 센서 기반 closed-loop이므로 누적 없음 |
| GraspTarget 설정 중 race condition | Low | target_mutex_ + atomic flag 패턴 (기존 검증됨) |

---

## 8. 파일 변경 목록 (최종)

```
# 신규 생성 (3 files)
ur5e_bringup/include/ur5e_bringup/controllers/demo_wbc_controller.hpp
ur5e_bringup/src/controllers/demo_wbc_controller.cpp
ur5e_bringup/config/controllers/demo_wbc_controller.yaml

# 테스트 (1 file)
ur5e_bringup/test/test_demo_wbc_controller.cpp

# 수정 (3 files, minor changes)
ur5e_bringup/src/controllers/controller_registration.cpp  (+3 lines)
ur5e_bringup/CMakeLists.txt                               (+~15 lines)
ur5e_bringup/package.xml                                   (+1 line)

# 문서 업데이트 (2 files)
ur5e_bringup/README.md
CLAUDE.md
```
