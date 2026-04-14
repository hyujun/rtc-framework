# Phase 4B: DemoWbcController — 후속 작업

> **작성일**: 2026-04-14
> **선행**: Phase 4A (완료) — FSM skeleton + TSID 적분 파이프라인 + MVP 4 phase
> **목표**: Contact 연동, Retreat/Release 구현, 시뮬레이션 통합 검증

---

## 1. 구현 범위

Phase 4A MVP에서 skeleton만 구현한 contact-dependent phase 및 시뮬레이션 통합을 완성한다.

### 1.1 Phase 4A에서 완료된 것 (재확인)

- [x] FSM skeleton: 8 phase 전체 enum + UpdatePhase() 전환 조건 코드
- [x] **kIdle**: position hold
- [x] **kApproach**: joint-space quintic trajectory (arm + hand 독립)
- [x] **kPreGrasp**: TSID QP → acceleration → semi-implicit Euler → position
- [x] **kFallback**: QP 연속 실패 시 position hold
- [x] PinocchioCache + TSIDController 초기화
- [x] Joint reorder (device ↔ Pinocchio order)
- [x] E-STOP interface
- [x] Gains layout (7 values)
- [x] 13 unit tests
- [x] Build + test gate 통과

### 1.2 Phase 4B에서 완성할 것

- [ ] **Sensor integration** (`ReadState`): fingertip force/contact data 파싱
- [ ] **kClosure**: contact 활성화 + ForceTask 활성화 + 접촉력 형성
- [ ] **kHold**: grasp 유지 + anomaly monitoring (slip, deformation)
- [ ] **kRetreat**: contact 비활성화 + 후퇴 trajectory
- [ ] **kRelease**: hand open trajectory
- [ ] **Contact frame registration** in PinocchioCache (fingertip Jacobians)
- [ ] **Simulation integration test** (MuJoCo)
- [ ] **Performance profiling**: QP solve time @ 16-DoF < 200μs 확인

---

## 2. 세부 작업 항목

### 2.1 Sensor Integration (`ReadState`)

현재 `ReadState()`는 no-op. DemoJointController의 ReadState 패턴을 이식:

```cpp
// Parse fingertip F/T inference data per fingertip
//   force_magnitude[f] = ‖force[0..2]‖
//   contact_flag[f]    = inference_data[ft_base]
//   displacement[f]    = inference_data[ft_base + 4..6]
// → Store in FingertipSensorData array (member)
```

**파일**: `ur5e_bringup/src/controllers/demo_wbc_controller.cpp`
**참조**: `DemoJointController::ReadState()` (lines 155-201)

**테스트**: fingertip 센서 데이터 주입 → ReadState → 파싱 결과 검증

### 2.2 kClosure Phase

**역할**: TCP가 pre-grasp 위치에 도달 후 fingertip을 object에 접촉시킴.

**전환 조건** (이미 UpdatePhase에 코드 존재):
- `kPreGrasp → kClosure`: `ComputeTcpError(tcp_goal_) < epsilon_pregrasp_`

**OnPhaseEnter(kClosure)**:
- [x] Phase preset "closure" 적용 (pre-resolved)
- [x] Contact activation (`c.active = true`)
- [ ] ForceTask reference 설정 (per-fingertip target force)
- [ ] Hand joint target을 close 위치로 ramp 시작

**전환 조건 추가**:
- `kClosure → kHold`: fingertip contact 감지 (force > threshold, 최소 N finger)

```cpp
// In UpdatePhase(kClosure):
int active_contacts = 0;
for (int f = 0; f < num_fingertips_; ++f) {
    if (fingertip_data_[f].valid &&
        fingertip_data_[f].force_magnitude > force_contact_threshold_) {
        ++active_contacts;
    }
}
if (active_contacts >= min_contacts_for_hold_) {
    next = WbcPhase::kHold;
}
```

### 2.3 kHold Phase

**역할**: grasp 유지 + 외부 disturbance 보상.

**OnPhaseEnter(kHold)**:
- [x] Phase preset "hold" 적용
- [x] Contact 유지 (already active from kClosure)
- [ ] ForceTask reference = `gains_.grasp_target_force`

**Anomaly detection** (매 tick):
- Slip detection: `|d force / dt| > slip_threshold_`
- Deformation guard: fingertip displacement > threshold
- 이상 감지 시 → kFallback 또는 re-grasp 시도

**전환 조건** (이미 UpdatePhase에 코드 존재):
- `kHold → kRetreat`: `grasp_cmd == 2`

### 2.4 kRetreat Phase

**역할**: contact 해제 후 안전 위치로 후퇴.

**OnPhaseEnter(kRetreat)**:
- [x] Contact 비활성화
- [ ] Retreat joint-space target 계산
  - 옵션 A: approach 시점의 q_start를 저장했다가 사용
  - 옵션 B: 현재 q에서 +Z 방향 offset 적용한 IK
- [ ] Quintic trajectory 초기화 (current → retreat target)

**전환 조건** (이미 UpdatePhase에 코드 존재):
- `kRetreat → kRelease`: trajectory complete

### 2.5 kRelease Phase

**역할**: hand 열기.

**OnPhaseEnter(kRelease)**:
- [ ] Hand open joint target 설정 (모든 finger flex joint → 0)
- [ ] Hand trajectory 초기화

**전환 조건** (이미 UpdatePhase에 코드 존재):
- `kRelease → kIdle`: trajectory complete

### 2.6 Contact Frame Registration

PinocchioCache가 contact Jacobian을 계산하려면 fingertip frame이 등록되어야 함. 현재 `contact_mgr_config_.load()`에서 frame_id resolve는 되지만, 추가 검증 필요:

```cpp
// In LoadConfig after pinocchio_cache_.init():
// Verify all contact frames are in pinocchio_cache_.contact_frames
for (const auto& c : contact_mgr_config_.contacts) {
    if (c.frame_id == 0) {
        RCLCPP_ERROR(logger_, "Contact '%s' frame '%s' not found",
            c.name.c_str(), c.frame_name.c_str());
    }
}
```

### 2.7 ForceTask Integration

현재 Phase 4A YAML에는 ForceTask가 정의되지 않음 (`tasks` 섹션에 `force`가 주석). Phase 4B에서 활성화:

```yaml
# In demo_wbc_controller.yaml, tsid.tasks:
force:
  type: "force"
  weight: 10.0
  priority: 1
```

그리고 phase_preset "closure", "hold"에 `force: { active: true }` 추가.

**Runtime reference 설정** (OnPhaseEnter kClosure/kHold):

```cpp
auto* force_task = tsid_controller_.formulation().get_task("force");
if (force_task) {
    // Set per-contact force target
    Eigen::VectorXd lambda_des = Eigen::VectorXd::Zero(
        contact_mgr_config_.max_contact_vars);
    // For each active contact: set +Z force of gains_.grasp_target_force
    for (int i = 0; i < contact_state_.active_count; ++i) {
        // contact_dim=3 for point contacts
        const int offset = i * 3;
        lambda_des[offset + 2] = gains_.grasp_target_force;  // +Z normal
    }
    static_cast<rtc::tsid::ForceTask*>(force_task)->set_force_references(
        lambda_des);
}
```

### 2.8 Friction Cone Constraint

Contact force의 물리적 실현 가능성을 보장하려면 friction cone constraint 추가:

```yaml
# In demo_wbc_controller.yaml, tsid.constraints:
friction_cone:
  type: "friction_cone"
  n_faces: 8
```

그리고 phase_preset "closure", "hold"에 `friction_cone: { active: true }` 추가.

---

## 3. 시뮬레이션 통합 테스트

### 3.1 Launch 설정

`ur5e_bringup/launch/sim.launch.py`에 `initial_controller` 파라미터로 `demo_wbc_controller` 선택 가능하도록 검증:

```bash
ros2 launch ur5e_bringup sim.launch.py initial_controller:=demo_wbc_controller
```

### 3.2 테스트 시나리오

1. **Idle**: 기동 후 home pose 유지 확인 (10초)
2. **Approach**: `/ur5e/joint_goal` 토픽으로 pre-grasp 타겟 발행
   - `~/controller_gains`에 `grasp_cmd=1` 발행
   - FSM: kIdle → kApproach → kPreGrasp 전환 로그 확인
3. **PreGrasp**: TSID QP solve time 측정 (< 200μs 기대)
4. **Closure → Hold**: hand joint target + force target으로 grasp 시도
   - MuJoCo object와의 contact 확인
5. **Retreat → Release**: `grasp_cmd=2`로 release
   - FSM: kHold → kRetreat → kRelease → kIdle

### 3.3 성능 측정

- TSID solve time: `RCLCPP_INFO_THROTTLE`로 로깅 (이미 구현됨)
- Position continuity: phase 전환 시 commanded position jump 없음
- Joint limit: TSID JointLimitConstraint로 보호 확인
- QP 실패 빈도: 0이 정상, 반복적 실패 시 preset/gain 튜닝 필요

---

## 4. 잠재적 이슈 및 완화 전략

| 이슈 | 완화 |
|------|------|
| 16-DoF QP solve > 2ms (500Hz budget 초과) | ProxSuite tuning, HQP → WQP 유지, warm-start 활용 |
| Pinocchio joint ordering과 DeviceState ordering 불일치 | 이미 `BuildJointReorderMap()`으로 해결. 단위 테스트 추가 필요 |
| Contact 활성/비활성 전환 시 position discontinuity | `OnPhaseEnter(kClosure)`에서 `q_next_ = q_curr_` 재설정 (이미 구현) |
| MuJoCo contact sensor와 실제 force inference 차이 | 시뮬레이션에서는 MuJoCo contact force를 직접 사용, 실제에서는 ONNX inference |
| ForceTask가 position 제어에서 실제 force tracking 실패 | Phase 4B에서 확인. 필요 시 force target을 hand joint target으로 변환 (indirect force control) |

---

## 5. 검증 기준

Phase 4B 완료 조건:

- [ ] Sensor data 파싱 단위 테스트 통과
- [ ] 모든 8 phase의 OnPhaseEnter() 로직 구현
- [ ] 모든 phase 전환 조건이 실제로 트리거됨 (시뮬레이션 로그)
- [ ] MuJoCo 시뮬레이션에서 pick-and-place 1 cycle 성공
- [ ] TSID QP solve time < 500μs (typical), < 1ms (worst)
- [ ] 100회 연속 cycle 동안 kFallback 진입 없음
- [ ] Joint limit 위반 없음
- [ ] README.md 업데이트 (FSM 전체 설명 + 사용 예시)
- [ ] CLAUDE.md Controllers 테이블 업데이트 (MVP → full)

---

## 6. 예상 변경 파일

```
# 주요 수정
ur5e_bringup/src/controllers/demo_wbc_controller.cpp   (+~200 lines)
  - ReadState() sensor parsing
  - OnPhaseEnter() kClosure/kHold/kRetreat/kRelease
  - ForceTask reference 설정
  - Anomaly detection (kHold)

ur5e_bringup/include/ur5e_bringup/controllers/demo_wbc_controller.hpp  (+~30 lines)
  - FingertipSensorData struct (DemoJoint와 동일)
  - num_fingertips_, fingertip_data_ 멤버
  - min_contacts_for_hold_, slip_threshold_ 등 FSM 파라미터

ur5e_bringup/config/controllers/demo_wbc_controller.yaml
  - ForceTask + FrictionCone 활성화
  - phase_preset closure/hold에 force task 추가

# 테스트 추가
ur5e_bringup/test/test_demo_wbc_controller.cpp   (+~100 lines)
  - Sensor parsing test
  - Phase transition test (kClosure, kHold)
  - Contact activation test

# 문서
ur5e_bringup/README.md
  - DemoWbcController section 추가 (architecture, FSM, gains, 사용법)
CLAUDE.md
  - Controllers 테이블 업데이트 (MVP 표기 제거)
```

---

## 7. 참고 자료

- `docs/phase4_critical_review.md`: Phase 4A 분석 + 아키텍처 결정
- `rtc_tsid/README.md`: TSID API 레퍼런스
- `ur5e_bringup/src/controllers/demo_joint_controller.cpp`: sensor parsing 참조 패턴
- `rtc_controllers/src/grasp/grasp_controller.cpp`: contact detection 로직 참조
