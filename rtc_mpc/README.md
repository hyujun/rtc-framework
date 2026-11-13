# rtc_mpc

soft-RT MPC thread 와 hard-RT 제어 루프를 잇는 MPC ↔ RT 인터페이스 라이브러리. **유형**: 라이브러리 패키지

## 개요

Robot-agnostic 라이브러리다. soft-RT MPC thread (일반적으로 20 Hz) 와 hard-RT 제어 루프 (`control_rate`, default 500 Hz, 설계 범위 100 Hz–5 kHz) 사이의 배관을 제공한다: lock-free solution 전달, cubic Hermite 궤적 보간, Riccati 피드백, MPC thread skeleton. 구체 solver 통합 (Aligator ProxDDP) 은 `rtc::mpc::MPCThread` + `PhaseManagerBase` 에 꽂는다.

`rtc_tsid` 에는 의존하지 않는다. 소비자인 컨트롤러 (예: `integrated_bringup::DemoWbcController`) 가 MPC reference 를 TSID 태스크에 주입한다. 운영 기본 엔진은 `handler` 다.

## 빠른 시작

빌드·테스트는 [루트 README](../README.md#빠른-시작) 를 따른다. 이 패키지만 빌드하려면 다음과 같다. `fmt` · `pinocchio` · `aligator` · `mimalloc` 은 `<rtc_ws>/deps/install/` 에 소스 빌드된 것을 쓴다 ([build_deps.sh](../repo_scripts/scripts/build_deps.sh)).

```bash
./build.sh --tests -p rtc_mpc
```

MPC 를 기동하는 최소 흐름은 `MPCFactory::Create(cfg, model, initial_ctx, &handler)` 로 handler 를 만들고, `PhaseManagerBase` 구현과 함께 `HandlerMPCThread` 에 물려 thread 를 시작하는 것이다. 실제 사용례는 `test/test_mpc_thread_integration.cpp` 와 `integrated_bringup` 의 `DemoWbcController` 를 본다.

## 인터페이스

공개 헤더는 `include/rtc_mpc/<모듈>/` 아래에 있다. 시그니처·계약은 헤더 주석이 SSoT 다.

| 모듈 | 헤더 | 역할 |
|---|---|---|
| `types/` | `mpc_solution_types.hpp` | `MPCSolution` · `MPCStateSnapshot` (trivially copyable, 고정 용량 — free-flyer + 관절 + 접촉 변수 상한, `SolverStatus` · 잔차 · provenance 포함) |
| `types/` | `wbc_reference.hpp` | `WbcReference` — 하류 whole-body 컨트롤러에 넘기는 tick 당 참조 POD (규약은 헤더) |
| `types/` | `contact_plan_types.hpp` | `ContactFrameInfo` · `ContactPhase` · `ContactPlan` (non-RT, OCP 빌드 경로) |
| `model/` | `robot_model_handler.hpp` | `pinocchio::Model` 래퍼 + YAML frame 해석 (부동 베이스 감지, kinodynamic `nu`) |
| `model/` | `wb_layout.hpp` | 전신 상태 · 입력 layout 한 곳 — free-flyer `[p, q_xyzw]` · LOCAL twist · `[W_1 … W_nc; q̈_j]` · tangent 상태 차 (`StateDifference`) |
| `dynamics/` | `kinodynamic_dynamics.hpp` | 부동 베이스 kinodynamic ODE 와 stage 이산화 (`semi_implicit_euler` · `rk2`) 생성 · stage 안 사본의 접촉 mode 를 제자리 전환 (`OdeOf` · `SetMode`, 할당 0) |
| `phase/` | `phase_manager_base.hpp` | 순수 가상 FSM 경계. 구체 구현은 하위 bringup 패키지가 갖는다 |
| `phase/` | `phase_cost_config.hpp` | cost POD + no-throw YAML factory |
| `phase/` | `phase_context.hpp` | manager → OCP builder 묶음 (dispatch key `ocp_type` · `horizon`) |
| `ocp/` | `ocp_handler_base.hpp` | 추상 OCP builder (`Build` / `UpdateReferences`) · `control_model()` (joint torque / kinodynamic wrench — handler 가 이 값으로 base 를 거부 · `u` 를 pack) · `cold_seed()` |
| `ocp/` | `contact_{kinematics,tracking_residual}.hpp` | 접촉 frame 기구학 · 추종 residual (가속도는 stage ODE) |
| `ocp/` | `contact_constraints.hpp` · `barrier_costs.hpp` | mode 무관 고정 행 접촉 등식 (stance · swing) · 마찰 · CoP · joint limit · 구 쌍 충돌 barrier |
| `ocp/` | `kinodynamic_ocp{,_config}.hpp` | kinodynamic OCP · `mpc.kinodynamic:` 로더 (`horizon` 의 mode · 참조를 할당 없이, cold seed 는 노드 `u_ref`) |
| `ocp/` | `cost_factory.hpp` | stage 별 `aligator::CostStack` 생성 (weight-gated, no-throw) |
| `ocp/` | `contact_light_ocp.hpp` | `OCPHandlerBase` 구현, dispatch key `"contact_light"` |
| `ocp/` | `contact_rich_ocp.hpp` | 접촉력 cost + 마찰 원뿔 부등식 추가, key `"contact_rich"` (cold start 는 호출자가 seed) |
| `ocp/` | `grasp_quality_provider.hpp` | grasp-quality residual 확장 seam (구체 provider 는 아직 없다) |
| `comm/` | `triple_buffer.hpp` | lock-free triple buffer, zero-copy consumer |
| `interpolation/` | `trajectory_interpolator.hpp` | OCP 노드 사이 cubic Hermite 보간 |
| `feedback/` | `riccati_feedback.hpp` | `u_fb = gain_scale · K · Δx` (accel-only 모드 선택) · 평탄화된 gain layout 의 writer `PackRiccatiGainRowMajor` ↔ reader `SetGain` |
| `manager/` | `mpc_solution_manager.hpp` | TripleBuffer + Interpolator + Feedback + SeqLock facade |
| `thread/` | `mpc_thread.hpp` | `PeriodicRtThread` 기반 solve loop. 서브클래스가 `Solve(state, out)` 구현, `MockMPCThread` 는 테스트용 |
| `thread/` | `handler_mpc_thread.hpp` | `PhaseManagerBase` FSM 과 `MPCHandlerBase` solver 를 묶는 구체 thread |
| `handler/` | `mpc_handler_base.hpp` | solve orchestrator (`Init` / `Solve` / `SeedWarmStart`), `MPCSolverConfig` |
| `handler/` | `contact_light_mpc.hpp` · `contact_rich_mpc.hpp` | 각 OCP 를 감싼 구체 handler |
| `handler/` | `mpc_factory.hpp` | `ocp_type` 으로 dispatch 하는 YAML factory (`MPCFactory::Create`) |
| `handler/` | `kinodynamic_mpc.hpp` | `KinodynamicOCP` 의 handler |
| `reference/` | `command_filter.hpp` | 보행 velocity 명령 필터 (non-RT, 고정 주기 — 체인은 헤더) → `FilteredCommand` POD |
| `reference/` | `gait_table.hpp` · `gait_scheduler.hpp` | YAML gait 표 (접촉 비트마스크, 전환 문턱) · 창 tiling 과 전환 정책 (MPC thread, 할당 0) |
| `reference/` | `contact_schedule.hpp` | `ContactSchedule` POD (mode 변화 event · `schedule_id` · `epoch`) + `ModeAt` 등 helper |
| `reference/` | `swing_trajectory.hpp` | swing 발 z 참조 (두 구간 cubic Hermite, apex 50 %, 짧은 swing 은 축소) + impact proximity — YAML `swing:` · `terrain:` 로더, `BuildSwing` · `Evaluate` 는 할당 0 |
| `reference/` | `surface_frame.hpp` | `SurfaceFrame` — 발이 닿는 world 평면 (지금은 flat 값) 과 `HeightAt` |
| `reference/` | `stage_reference.hpp` | `StageReference` · `HorizonReference` POD — 노드별 q · v (base 3 knot) · 접촉 pose · twist · wrench · impact proximity (padding 없음, 두 slot) |
| `reference/` | `stage_reference_builder.hpp` | `StageReferenceBuilder` (MPC thread, 할당 0) — scheduler 창 + snapshot + 명령 → horizon, 완성 뒤 slot swap · YAML `reference:` (q_default · arm swing) |
| `logging/` | `mpc_timing_logger.hpp` | `<session>/timing/mpc_timing_log.csv` writer (아래 관측성) |

### 설계 약속

- Robot-agnostic: 고정 DoF · 로봇 이름 · frame 문자열 하드코딩이 없다. 토폴로지는 `pinocchio::Model` + YAML 에서만 온다 (ARCH-1).
- RT 데이터는 trivially copyable 이다: `MPCSolution` · `MPCStateSnapshot` · `WbcReference` 는 SeqLock / TripleBuffer 로 건너가므로 동적 멤버가 없다. `TripleBuffer` 슬롯과 보간기의 사본은 생성 · Init 때 heap 에 한 번 잡고 (MPC thread 의 scratch · 이전 해는 인라인), 매 solve 는 Riccati gain 을 앞 노드 몇 개만 싣는다.
- 계약 (타입 · layout · Riccati) 은 부동 베이스를 포함하지만 **보간은 fixed-base 만**이다: cubic Hermite 는 Euclidean `q` 를 가정하므로 부동 layout 으로 `Init` 된 manager 는 `ComputeReference` 를 거부한다 (`InterpMeta::floating_unsupported`).
- `RobotModelHandler::Init` 은 throw 하지 않고 `RobotModelInitError` 를 돌려준다.
- `end_effector_frame` (또는 목록 `end_effector_frames`) 과 `base_frame` 이 필수다. `base_frame` 누락은 `kInvalidYamlSchema`, 해석 불가 이름은 `kMissingBaseFrame` 이며 world 기준은 `base_frame: "universe"` 로 명시한다. 부동 베이스 모델은 `universe` 만 받는다 (`kBaseFrameNotUniverseOnFloating`). `PhaseContext::ee_target` 은 `base_frame` 기준이다.

### 관측성

MPC thread 는 RT 컨텍스트라 `HandlerMPCThread::Solve` 는 I/O 를 하지 않는다. 실패 경로는 lock-free atomic (`FailedSolves()` · `TotalSolves()` · `LastSolveErrorCode()` · `LastPhaseId()` · `NullHandlerHit()`) 만 올리고, 소유 컨트롤러의 non-RT 1 Hz aux timer 가 증분을 보고한다. 같은 timer 가 `MPCThread::TimingProducer()` 의 per-tick SPSC 를 CSV 로 비운다.

CSV 는 MPC tick 마다 한 행이고 CM RT loop 와 같은 8열 스키마다: `t_wall_ns,tick_count,run_id,t_state_us,t_compute_us,t_publish_us,t_total_us,jitter_us`. `Solve` 가 계속 실패해도 행은 계속 늘어난다 (publish 시간만 0). consumer 가 뒤처지면 `DropCount()` 가 올라간다. cross-mode swap 은 이 thread 에서 할당하는 기록된 알려진 위반이다 ([invariants.md](../agent_docs/invariants.md) §RT Path Invariants).

## 설정

[config/mpc_default.yaml](config/mpc_default.yaml) 은 robot-agnostic 참조 템플릿이다. `<...>` placeholder 는 로봇별 설정에서 덮어쓰며, 값은 `PhaseCostConfig::LoadFromYaml` 과 `RobotModelHandler::Init` 이 읽는다. 기본값은 파일이 갖는다. 로봇별 실제 설정은 `integrated_bringup/config/<robot>/controllers/mpc/` 에 있다. `reference/` 로더의 템플릿은 [config/reference_default.yaml](config/reference_default.yaml) 이다 (모든 키 필수).

`mpc.solver:` 키 (standing · online iteration 예산, rollout, LQ backend, thread 수) 의 스키마는 `mpc_factory.hpp` 가 갖는다. step acceptance (non-monotone line search) 와 Hessian 근사 (Gauss-Newton) 는 Aligator 기본값이며 키가 없다.

## 문제 해결

| 증상 | 원인 | 조치 |
|---|---|---|
| `contact_rich` solve 테스트가 `free(): invalid pointer` 로 죽는다 | pinocchio/aligator ABI 경계에서 mimalloc 이 할당한 것을 glibc 가 해제한다 (Pinocchio 4.0) | `CMakeLists.txt` 가 aligator solve gtest 마다 `LD_PRELOAD` 로 mimalloc 을 건다. 이 설정을 끄거나 gtest 바이너리를 직접 실행하지 않고 `colcon test` 로 돌린다 |
| mimalloc 을 못 찾는다는 configure 경고 | `RTC_DEPS_PREFIX` 또는 `aligator_DIR` 상대 `lib/` 에 없다 | `repo_scripts/scripts/build_deps.sh` 로 deps 를 빌드하고 `setup_env.sh` 를 source 한다 |
| Debug 빌드에서 proxsuite assert | all-zero C assert 가 Debug 에서 표면화 | 이 패키지의 coverage·측정은 Release 로 한다 ([testing-rationale.md](../docs/reference/testing-rationale.md) §Coverage 함정) |

mimalloc `LD_PRELOAD` 의 경로 해석(`RTC_DEPS_PREFIX` → `aligator_DIR`-상대 `lib/`)은 `CMakeLists.txt` 가 갖는다.

## 더 읽기

- 테스트 명령·카운트 측정: [docs/testing.md](../docs/testing.md) §Test 측정
- MPC thread 의 실행 컨텍스트: [architecture.md](../agent_docs/architecture.md) §Execution Contexts
- source-build deps: [루트 README](../README.md#빠른-시작)

## 의존성

[package.xml](package.xml) · MIT. `rtc_base` (SeqLock · threading) · `rtc_urdf_bridge` (free-flyer root 판정 `IsFreeFlyerRoot`, `rtc_tsid` 와 공유) · Eigen3 · yaml-cpp · Pinocchio 4.0 (collision 백엔드 coal 은 pinocchio 가 끌어온다) · fmt ≥ 10 · Aligator 0.19.x (ProxDDP). `fmt` · `pinocchio` · `aligator` 는 `deps/install/` 에서 `CMAKE_PREFIX_PATH` 로 찾으므로 `*_DIR` 힌트가 필요 없다.
