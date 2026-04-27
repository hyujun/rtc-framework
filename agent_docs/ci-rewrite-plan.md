# ROS 2 Advanced CI Rewrite Plan

**Status**: Phase 0 (analysis frozen, decisions locked) — 2026-04-27
**Owner**: junho.park
**Driver**: 최근 10 run 중 5 run 실패. root cause = 워크스페이스 격리(2026-04-21) 이후 CI 가 `deps/install/` (fmt 11.1.4 / mimalloc / aligator 0.19.0) 빌드 단계를 누락.

## 1. 실패 원인 분석 (2026-04-27 기준 최근 5 fail)

### 1.1 Primary: fmt 버전 불일치 (Build & Test, Clang-Tidy, CodeQL 공통)

```
CMake Error at CMakeLists.txt:35 (find_package):
  Could not find a configuration file for package "fmt" that is compatible
  with requested version "10".
    /usr/lib/x86_64-linux-gnu/cmake/fmt/fmt-config.cmake, version: 9.1.0
Failed   <<< rtc_mpc [3.94s, exited with code 2]
```

- `rtc_mpc/CMakeLists.txt:35` → `find_package(fmt 10 REQUIRED)`
- Ubuntu 24.04 apt `libfmt-dev` = 9.1.0
- 의존성 chain: `ur5e_bringup → rtc_mpc → fmt/aligator`
- `BUILD_PACKAGES` 의 `--packages-up-to ur5e_bringup` 가 `rtc_mpc` 를 transitive 로 끌어들임
- **현 CI workflow 에 `build_deps.sh` 호출 단계 부재** ([rtc_scripts/scripts/build_deps.sh](../rtc_scripts/scripts/build_deps.sh))

### 1.2 Secondary: CodeQL OOM-mitigation swap step 충돌

```
fallocate failed: Text file busy
##[error]Process completed with exit code 1.
```

- `dd` / `fallocate` 가 일부 hosted runner image 에서 race condition 으로 실패
- root cause 와 무관하게 CodeQL 단독 실패 야기

### 1.3 Tertiary: Python Test exit 5 (24958368197 — `aa4cc8f` 직후)

- pytest exit 5 = "no tests collected"
- `rtc_tools` `plot_rtc_log.py` 분리 리팩토링(2102→194 lines, [aa4cc8f](https://github.com/hyujun/rtc-framework/commit/aa4cc8f)) 시 테스트 파일 위치/패턴 변경 가능성

### 1.4 누적된 구조 부채

| # | 항목 | 영향 |
|---|------|------|
| S-1 | `BUILD/CPP_TEST/PYTHON_TEST/LINT_PACKAGES` 4벌 하드코딩 (env block) | drift, 패키지 추가 시 4곳 수정 |
| S-2 | `Generate Coverage Report` step 이 빌드 실패 시에도 lcov 호출 | 이중 실패, 노이즈 |
| S-3 | `if: always()` + `set -e` 미동기 | 진단 step 이 본질 실패 신호 가림 |
| S-4 | `claude-review` job 이 `continue-on-error: true` | PR 마다 항상 skip 표시 → 신호 0 |
| S-5 | `FORCE_JAVASCRIPT_ACTIONS_TO_NODE24: true` 임시 우회 | 2026-06-02 강제 전환 |
| S-6 | 691 줄 단일 yaml, 6 job × 보일러플레이트 반복 | 유지보수 비용 |

## 2. 결정 (locked, 사용자 컨펌 2026-04-27)

| ID | 항목 | 결정 | 이유 |
|----|------|------|------|
| D-1 | claude-review job | **제거** | continue-on-error → 신호 0. PR 리뷰는 별도 트리거로 |
| D-2 | rtc_mujoco_sim CI | **미포함** | MuJoCo 미설치, optional dep |
| D-3 | OS / ROS matrix | **Ubuntu 24.04 + jazzy 단일** | Humble 추가 시 CI 시간 2x, 현 운영 환경 jazzy |
| D-4 | deps 공유 메커니즘 | **artifact** | cross-job 공유 명확. cache 보다 결정적 |
| D-Q2 | rtc_mpc CI 포함 | **포함 (build_deps 빌드)** | ur5e_bringup transitive — 분리 시 ur5e_bringup 도 제외해야 함 |
| D-Q3 | PR 분할 | **Phase 별 PR** | 단일 거대 PR 보다 회귀 격리 용이 |

**Locked**: 위 6개 결정은 rewrite 종료까지 재논의 X. 변경 필요 시 새 [CONCERN] 발의.

## 3. 목표 (acceptance criteria)

1. **CI = local build flow 1:1** — `install.sh` / `build_deps.sh` / `build.sh` 가 진실의 근원
2. **패키지 목록 SSoT** — workflow + agent_docs 가 한 파일 참조
3. **빌드 실패 = job 실패** — 진단 step 이 본질 실패를 덮지 않음
4. **Job 책임 단일** — 한 job 망가지면 한 가지 신호
5. **Wall-clock < 8 min** (deps cache hit 시) vs 현재 11~14 min
6. **Yaml 라인 < 450** (현재 691)

## 4. Phase 별 작업

### Phase 0 — 분석 / 동결 (이 문서)

- [x] root cause 기록
- [x] 결정 고정 (D-1~D-4, D-Q2, D-Q3)
- [x] 현 yaml 백업 git history 로 충분 (별도 .bak 불요)

### Phase 1 — Composite actions (PR 1)

`.github/actions/` 하위:

| Action | 입력 | 책임 |
|--------|------|------|
| `setup-rtc-env` | `ros-distro`, `cache-version`, `extra-apt` | setup-ros + apt cache + colcon upgrade + numpy fix |
| `build-isolated-deps` | `ros-distro` | `build_deps.sh` 실행 + `deps/install/` 결과 노출 (artifact name 출력) |
| `colcon-build` | `packages`, `cmake-args` | colcon build 표준화 |
| `colcon-test-report` | `packages`, `summary-title` | colcon test + ctest fallback dump + GITHUB_STEP_SUMMARY |

각 action 은 자체 README 1~2 단락 포함.

### Phase 2 — 패키지 목록 SSoT (PR 2)

`.github/ci-packages.yml`:

```yaml
# Single source of truth for CI package selection.
# When adding a new package: update this file + agent_docs/architecture.md table.
build:
  - rtc_controller_manager
  - rtc_controllers
  - rtc_tsid
  - ur5e_bringup
  - ur5e_bt_coordinator
  - ur5e_hand_driver
  - rtc_urdf_bridge
  - shape_estimation
test_cpp:
  - rtc_base
  - rtc_controllers
  - rtc_tsid
  - rtc_urdf_bridge
  - shape_estimation
  - ur5e_bringup
  - ur5e_bt_coordinator
  - ur5e_hand_driver
test_python:
  - rtc_tools
  - rtc_digital_twin
lint_cpp:
  - rtc_base
  - rtc_communication
  - rtc_controller_interface
  - rtc_controller_manager
  - rtc_controllers
  - rtc_inference
  - rtc_msgs
  - rtc_tsid
  - rtc_urdf_bridge
  - shape_estimation
  - ur5e_bringup
  - ur5e_bt_coordinator
  - ur5e_hand_driver
exclude:
  - rtc_mujoco_sim   # D-2: MuJoCo not installed in CI
  - rtc_mpc          # transitive only via ur5e_bringup; never directly tested in CI
  - rtc_digital_twin # python only; covered in test_python
```

workflow 에서 yq 로 parse → step output 으로 propagate.

`agent_docs/architecture.md` 패키지 표에서 이 파일 link.

### Phase 3 — Job 재구성 (PR 3, 핵심)

```
detect-changes
   ↓
build-deps                  ← 신규: fmt + mimalloc + aligator → artifact
   ↓ (download artifact)
[병렬]
├── build-test (jazzy)
├── clang-tidy
├── codeql                  ← swap step 제거 검토 (또는 actions/setup-swap 검증)
├── lint-cpp (cppcheck)     ← deps 불요
└── python-test             ← deps 불요

build-test ─► coverage-upload  (별도 job, success() 게이트)
codeql     ─► (자체 SARIF upload 유지)
```

핵심 변경:
- **`build-deps` job 신설** — `build_deps.sh` 실행 → `deps/install/` tarball artifact (`deps-install-jazzy`)
  - 캐시 키: `hashFiles('deps.repos', 'rtc_scripts/scripts/build_deps.sh')`
  - 캐시 hit 시 actions/cache 만 복원 (artifact upload skip)
- **`coverage-upload` 분리** — build-test 가 lcov 실패로 죽지 않음. `needs: build-test, if: success()`
- **`continue-on-error` 제거** — clang-tidy 만 유지 (warning 신호용)
- **`claude-review` job 제거** — D-1
- **CodeQL swap step** — `fallocate` race 우회: `swapfile-maintainer-action` 또는 inline `if [ ! -f /swapfile ]; then ...` guard

### Phase 4 — 진단 강화 (PR 4)

| 항목 | 처리 |
|------|------|
| 빌드 실패 시 `log/latest_build/<pkg>/stderr.log` upload-artifact (`if: failure()`) | composite action `colcon-build` 내장 |
| `colcon test-result` → GITHUB_STEP_SUMMARY markdown 표 | composite action `colcon-test-report` 내장 |
| 빌드 실패 시 coverage skip | `coverage-upload` job 의 `if: needs.build-test.result == 'success'` |
| Job summary annotation (`::error title=...`) | failed test 만 (현재 grep 패턴 더 좁힘) |

### Phase 5 — 부가 정리 (PR 5)

| 항목 | 처리 |
|------|------|
| `actions/checkout@v4` → `@v5` | Node 24 native 검증 |
| `dorny/paths-filter@v3` → 최신 | Node 24 호환 확인 |
| `FORCE_JAVASCRIPT_ACTIONS_TO_NODE24` env | 모든 action Node 24 native 확정 후 제거 |
| `paths-ignore` 에 `agent_docs/**` 추가 | 문서만 변경 시 빌드 skip |
| codecov flags | `cpp`, `python` 분리 유지, distro suffix 제거 (단일 distro) |
| `LINT_PACKAGES` 의 `rtc_msgs` | `include`/`src` 없음 → 자동 skip 로직 유지 (별도 작업 X) |

### Phase 6 — 검증 (PR 6, optional)

회귀 매트릭스:

| 변경 | 기대 동작 |
|------|----------|
| `*.md` only | all jobs skipped (paths-ignore) |
| `rtc_tools/foo.py` | python-test only |
| `rtc_mpc/CMakeLists.txt` | build-deps + build-test + clang-tidy + codeql |
| `deps.repos` 변경 | build-deps cache miss → 전체 rebuild |
| `agent_docs/foo.md` only | all jobs skipped |

Wall-clock 측정: `gh run watch` × 3 회 평균. 목표 < 8 min (cache hit), < 14 min (cold).

## 5. 단계별 PR 순서

| # | PR 제목 | 의존 | 검증 |
|---|---------|------|------|
| 1 | `ci: add composite actions for ROS workflow primitives` | - | 기존 yaml 그대로, action 호출 X (no-op merge) |
| 2 | `ci: extract package list to .github/ci-packages.yml SSoT` | PR 1 | 기존 4벌 env 와 동일성 확인 |
| 3 | `ci: rewrite jobs — build-deps artifact, drop claude-review, split coverage` | PR 1, 2 | **fmt 11 빌드 통과** (root cause fix) |
| 4 | `ci: improve diagnostics — failure artifacts, gated coverage` | PR 3 | 의도 회귀 매트릭스 |
| 5 | `ci: cleanup — Node 24 native, agent_docs paths-ignore` | PR 3 | warning 0 |
| 6 | (optional) `ci: validation matrix doc` | PR 5 | wall-clock 측정 결과 기록 |

각 PR 은 별도 fork branch 에서 dry-run 후 main merge.

## 6. Risk register

| Risk | Mitigation |
|------|------------|
| `build_deps.sh` 가 hosted runner 에서 빌드 실패 (ROS hpp-fcl path) | Phase 1 에서 build-isolated-deps action 단독으로 dry-run 검증 |
| artifact 다운로드 시간이 cache 보다 느림 | 측정 후 cache 로 전환 가능 (key 는 동일) |
| Phase 3 PR 가 main 깨면 다른 작업 차단 | fork branch dry-run + draft PR + `gh run watch` 통과 확인 |
| `agent_docs/**` paths-ignore 가 너무 광범위 | `agent_docs/*.md` 로 좁힘 검토 |
| `rtc_mpc` 만 `--packages-up-to` 에서 빠지면 ur5e_bringup 도 빠져야 함 | D-Q2 결정대로 둘 다 포함 |

## 7. 완료 정의 (DoD)

- [ ] 6 개 PR 모두 main merge
- [ ] main 푸시 시 ROS2 Advanced CI 3 회 연속 success
- [ ] yaml line < 450
- [ ] `agent_docs/architecture.md` 패키지 표가 `.github/ci-packages.yml` 참조
- [ ] 본 문서 status 를 "completed" 로 갱신, 이후 archive 또는 삭제
