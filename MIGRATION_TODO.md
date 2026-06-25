# Pinocchio 4.0 — Known Issues & Follow-ups

Pinocchio 3.9 → 4.0 마이그레이션 후 남은 **durable** 항목. 시점-의존 진행 status 는
두지 않는다 (그건 git log / `~/.claude/plans/pinocchio-4-migration.md`). 아래 두 known
limitation 은 `rtc_mpc` 테스트 주석이 직접 참조한다.

## (a) ContactRich cross-mode cold-solve NaN (Risk #14, pin 4.0 악화)

cross-mode swap (ContactLight→ContactRich warm-seed) 의 첫 solve 가 aligator
`ALIGATOR_RAISE_IF_NAN` (merit-function.hxx:91, `d1 += Lxs[0]·dxs[0]` → NaN) 으로
`aligator::RuntimeError` → 우리 `catch(...)` 가 `kSolverException` 으로 변환.

- 근본: pin 4.0 재작성 `ConstraintCholesky`/`constraintDynamics` 수치가 기존 Risk #14
  cold-solve NaN 한계를 이 seed 에서 발현시킴.
- **seed-side fix 없음 (분석 완료):** proximal `mu` sweep (1e-10→1e-4) 전부 NaN;
  gravity-comp cold-seed fallback 도 NaN (ContactRich cold-solve 자체가 Risk #14 fragile —
  sibling `SolveWithGravityCompSeedAttempts` 는 try/catch+SUCCEED 로 *관용*만 함).
- **Production 영향:** `HandlerMPCThread::TryCrossModeSwap` 이 swap commit 후 첫 solve 가
  실패할 수 있음. per-tick graceful degradation (warn + 직전 solution hold). full-sim recovery
  미검증.
- **테스트 처리:** `test_mpc_factory.cpp` CrossModeSwap 테스트는 `kSolverDiverged` hard-gate
  유지 + 알려진 `kSolverException` 은 `GTEST_SKIP` (E-6 승인). upstream fix 시 복원.

## R1 — ConstraintCholesky::updateDamping RT alloc (upstream pin 4.0 regression)

`ConstraintCholeskyDecomposition::updateDamping(mu)` 가 매 `compute()` 마다
`m_damping = BlockDiagonalMatrix::ScalarIdentity(constraintDim(), mu)` 로 임시를 힙 할당
(`constraint-cholesky-def.hxx:422`, libpinocchio_default.so). aligator 가 stage 마다
`constraintDynamics` 호출 → ContactLight solve 가 ~180 alloc/tick. **alloc==free, leak 없음**,
MPC solve thread (kHz RT tick 아님). 3.9 는 zero-alloc 이었음.

- **테스트 처리:** `test_mpc_handler_alloc_tracer.cpp` ContactLight 가드를 `==0` 에서
  balanced-canary (`allocs==frees` + `<30000`) 로 완화 (E-6 승인). upstream fix 시 `==0` 복원.
- header 가 아니라 .so 컴파일이라 로컬 패치 불가 → upstream 필요.

## mimalloc (contact_rich double-free)

aligator solve 가 pinocchio/aligator 경계에서 allocator 교차 free → pin 4.0 에서
`free(): invalid pointer`. mimalloc 이 first-resolved allocator 로 interpose 해야 함.
`rtc_mpc/CMakeLists.txt` 가 aligator-solve 테스트에 ctest `ENVIRONMENT` 로 `LD_PRELOAD`
주입 (격리 우회 아님 — sanctioned workaround). runtime 배포 시 동일 preload 보장 필요.

## Upstream issue 초안 (제출 대상: github.com/stack-of-tasks)

**#1 pinocchio — updateDamping heap-allocates per compute()**
> `ConstraintCholeskyDecompositionTpl::updateDamping(const Scalar& mu)` (constraint-cholesky-def.hxx:422)
> 가 `m_damping = BlockDiagonalMatrix::ScalarIdentity(constraintDim(), mu)` 로 매 호출 임시
> BlockDiagonalMatrix 를 힙 할당. `compute()` 가 무조건 호출 → RT MPC inner-loop 에서 stage 당
> 할당. 3.x `ContactCholeskyDecomposition` 은 warmup 후 alloc-free. 기대: constraintDim 불변 시
> 기존 `m_damping` 버퍼 재사용.

**#2 aligator/pinocchio — constrained OCP cold-solve NaN more readily on pin 4.0**
> friction-cone + contact-force `MultibodyConstraintFwdDynamics` OCP 를 cold/cross-mode seed 로
> solve 시 merit-function.hxx:91 `ALIGATOR_RAISE_IF_NAN(d1)` 에서 iter0 throw. pin 3.9 수렴,
> 4.0 재작성 ConstraintCholesky/constraintDynamics 수치가 NaR 영역으로 이동. proximal mu
> (1e-10→1e-4) 무효. friction-cone/contact-force residual 의 zero-force gradient NaN 의심.

## 후속 cleanup (비차단)

- `model.velocityLimit` deprecated (rtc_tsid/src/types/wbc_types.cpp:85) → pin 4.0 신규 접근자 교체.
- 131-header `deprecated/` shim 의존 제거 — 신규 4.0 경로로 include 전면 교체.
- `-Dhpp-fcl_DIR` dead-code 제거 (build_deps.sh, rtc_mpc/CMakeLists.txt) — pin 4.0 은 coal 사용.
- pinocchio 4.0 재고정: `sudo apt-mark hold ros-jazzy-pinocchio` (우발적 변경 방지).
