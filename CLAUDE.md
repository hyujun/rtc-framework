@AGENTS.md

## Claude Code

[AGENTS.md](AGENTS.md) 가 헌법 전문이며 위 import 로 매 세션 로드된다 (규칙은 거기에만 산다). 이 절은 Claude Code 에만 있는 메커니즘을 헌법 절에 매핑한다.

- **Enforcement (AGENTS.md §2 · AGENTS.md §4)** — `.claude/hooks/`. 검사 범위·blocking 구분·allowlist·bound 의 SSoT 는 각 hook 헤더 주석
  - `format-code.sh` (PostToolUse): Edit/Write 한 파일만 clang-format / ruff — Bash 로 쓴 파일은 거치지 않는다
  - `verify-changes.sh` (Stop): AGENTS.md §4 "커밋 전에 직접 돌려야 하는 것" 중 **기계 판정 가능한 부분만** 자동 수행하고 hard failure 시 `exit 2` 로 차단 — 여집합은 modification-guide.md §Completion Checklist 로 직접 확인. 변경 집합은 **마지막 통과 커밋(watermark) 기준 `git diff` ∪ untracked** 이므로 turn 안에서 commit 한 변경도 검증된다
    - **빌드·테스트는 turn 끝에서 실행하지 않는다** — 변경 패키지가 현재 내용에 대한 green verdict 를 갖는지만 확인하고, 없으면 차단한다. verdict 는 turn 안에서 `.claude/hooks/verify-changes.sh --run` 이 만든다 (전 gate + `--tests` 빌드 + `colcon test`; timeout·launch 실패는 "미검증" 으로 실패). **최종 검증은 `colcon test` 를 따로 돌리지 말고 이것으로 한다** — 같은 suite 를 두 번 돌리지 않는다. 오래 걸리면 백그라운드로 띄우고 **끝난 뒤** turn 을 끝낸다
    - 백그라운드 작업: subagent·workflow·teammate 가 도는 동안의 turn 끝에서는 검사를 **미루고** (차단 없음·watermark 유지) 모두 끝난 첫 turn 끝에서 몰아서 검증한다 — 병렬 편집 중의 통과는 검증이 아니다. 같은 workspace 에서 colcon·`build.sh` 가 이미 돌면 `--run` 은 그 옆에서 빌드하지 않고, verdict 가 없는 turn 끝은 그 빌드를 기다리라고 **차단**한다. 그 설치본의 MuJoCo sim 이 돌면 `--run` 은 빌드를 거부하고 (sim 을 느리게 해 측정을 망치므로), turn 끝은 없는 verdict 를 **미룬다** (나머지 gate 는 그대로, watermark 유지) — shell 작업 자체는 검사를 미루지 않는다
    - 측정 hold: unit 마다 sim 을 새로 띄우는 평가는 **unit 사이에 sim 이 없는 틈**이 있고, 턴이 그 틈에서 끝나면 hook 이 verdict 를 요구해 `--run` 이 다음 unit 옆에서 돌게 된다. 그런 평가의 드라이버는 `repo_scripts/scripts/with_verify_hold.sh <드라이버>` 로 띄운다 — wrapper 가 `<workspace>/.rtc-verify-hold` 에 자기 줄을 두고, 그 프로세스가 사는 동안 sim 과 같은 규칙이 적용된다 (형식·stale 판정은 hook 의 `workspace_holds`)
    - 판정 재사용: working tree 가 **마지막 통과 때와 내용이 같으면** 아무것도 다시 돌리지 않으므로 `--run` 이 통과시킨 트리는 commit 해도 turn 끝이 비용 없이 통과한다. 패키지 디렉토리들의 내용이 통과 때와 같으면 그 verdict 가 유지된다 (repo 루트 문서와 **패키지 안의 `*.md`** 는 고쳐도 유지 — 단 `repo_scripts` 와 PROC-3 의 verdict 는 트리 전체가 key 라 그것도 무효; 패키지 안의 그 밖의 파일을 고치면 무효 — `--run` 을 다시 돌린다). `--run` 이 도는 동안 패키지를 고치면 그 패키지는 verdict 를 받지 못하고, 그동안의 commit 은 다음 호출이 채점한다. `RTC_VERIFY_NO_REUSE=1` 로 끈다. 실행별 소요 시간·모드는 `.git/rtc-verify-timing.log`
  - `log-instructions-loaded.sh` (InstructionsLoaded): 로드 기록만
- **Rules (AGENTS.md §3)** — path-scoped rule `.claude/rules/rt-path.md` (RT-1~10 요약·예외·대안) · `arch-source.md` (ARCH-1·2·3·4·6) · `arch-build-meta.md` (ARCH-2·5·7) 이 매칭 파일 편집 시 자동 로드된다 (로드 조건은 frontmatter glob 이 SSoT — 박제 금지). **rule 채널은 두 센서로 검증한다**: glob 매칭은 `validate_claude_rules.py` (변경된 rule 한정, 0건이면 차단), 실제 로드는 `.claude/instructions-loaded.log`. rule 을 새로 쓰거나 glob 을 고쳤으면 매칭 파일을 하나 열고 로그를 자기 `session_id` 로 걸러 발화를 확인한다 (읽는 법은 그 hook 헤더). rule 이 안 뜨는 실패는 증상이 "규칙이 조용히 없는 것" 뿐이다
- **Skills (AGENTS.md §4)** — 추가 작업은 `adding-component` skill 이 진입점 (P5 일반화 · ARCH-3 · spec 필요 여부 판정과 발화 게이트만; 절차 SSoT 는 modification-guide.md). 런타임 검증 레시피는 `verify` skill
- **Inferential review (AGENTS.md §5.5)** — code review → `/code-review` (PR 준비는 `/code-review ultra`, 현재 branch 또는 `<PR#>`; `/ultrareview` 는 deprecated alias) · security review → `/security-review` · 재사용·단순화 review → `/simplify` (버그 탐지는 `/code-review`). 어떤 변경이 어느 review 인지는 AGENTS.md §5.5 와 modification-guide.md §Inferential review 트리거가 갖는다
- **Spec·plan 저장 (AGENTS.md §6.5 · AGENTS.md §6.6)** — Sprint Contract spec 과 handoff 는 `~/.claude/plans/<task-slug>.md` (`## Spec` / `## Progress` / `## Handoff`) 에서 스스로 관리한다. cross-tool 인계는 git issue
- **Context handoff 메커니즘 (AGENTS.md §6.6)** — 능동 제안 트리거, `/rename`+`/clear` / `/compact <focus>` / subagent / `/btw` / fork 선택, 보존 우선순위는 user-level CLAUDE.md `# Context handoff policy` 가 SSoT. "동일 문제 2회 초과 교정 → `/clear`" (context 위생) 는 handoff.md 의 "3회 → 중단·escalate" 와 다른 축이며 공존한다
- **Housekeeping (AGENTS.md §11)** — memory save / prune / Harness pruning 신호 보고는 user-level CLAUDE.md `# Post-task housekeeping` 가 SSoT (RTC 신호 카테고리: grep false-positive, hook 오차단, agent_docs 간 규칙 중복 drift). 완료된 `~/.claude/plans/*.md` 는 복원 가능하면 삭제
