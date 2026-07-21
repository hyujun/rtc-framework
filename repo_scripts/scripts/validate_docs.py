#!/usr/bin/env python3
"""에이전트 문서 코퍼스 validator (issue #213).

배경 — 왜 이 스크립트가 있는가
------------------------------
이 저장소의 invariant 자가검사는 오랫동안 **조용히 통과**해 왔다. 탐지 정규식이
마크다운 표 셀 안에 살면서 파이프를 ``\\|`` 로 이스케이프하고 백슬래시가 이중화돼,
verbatim 실행하면 syntax error(exit 2) 또는 0-match(exit 1) 가 났다. 에이전트에게
둘 다 "위반 없음"으로 읽힌다. 동시에 문서를 검사할 경로 자체가 없었다 —
``verify-changes.sh`` 는 소스/셸 변경이 없으면 early-exit 하고,
``ros2-advanced-ci.yml`` 은 ``paths-ignore`` 로 문서를 통째로 건너뛴다.

그래서 이 validator 는 **문서가 코드를 잘못 가리키는 것**과 **탐지 패턴이 문서로
되돌아오는 것** 두 가지를 막는다. 정규식의 SSoT 는 ``.claude/hooks/verify-changes.sh``
이고, 문서는 그 검사를 이름으로 인용만 한다 (issue #213 결정 3).

검사 목록
---------
=======  ==========================================================
 D001    깨진 로컬 링크 (마크다운 링크 + YAML 주석 안의 repo 경로)
 D002    ``#Lnnn`` 라인 앵커 — 편집마다 drift 하므로 전면 금지
 D003    private / machine-local 경로 참조 (``~/.claude/...`` 등)
 D004    상시 로드 문서의 표 셀에 박힌 raw 탐지 정규식
 D005    존재하지 않는 heading 앵커
=======  ==========================================================

사용법
------
``python3 repo_scripts/scripts/validate_docs.py [--format github]``

결함이 하나라도 있으면 exit 1. CI 와 로컬에서 동일하게 동작한다.

이 스크립트는 ``repo_scripts/CMakeLists.txt`` 의 ``install(PROGRAMS ...)`` 에 넣지
않는다 — 소스 트리를 ``git ls-files`` 로 훑는 저장소 유지보수 도구라서 설치본은
의미가 없다 (같은 디렉토리의 RT 설정 스크립트들과 성격이 다르다).
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
import unicodedata
from dataclasses import dataclass
from pathlib import Path

# --- 스캔 범위 -------------------------------------------------------------
# issue #213 코멘트 2 (추가 A): dangling 참조는 agent_docs/ 밖에도 있다 —
# .github/workflows/ros2-advanced-ci.yml 와 .github/actions/README.md 가 삭제된
# agent_docs/ci-rewrite-plan.md 를 아직 참조한다. 범위를 agent_docs/ 로 좁히면
# 이걸 놓치므로 저장소 전체를 본다.
SKIP_DIRS = {".git", "build", "install", "log", "node_modules", ".venv", "__pycache__"}

# D004 는 **invariant 탐지 패턴** 만 겨냥한다. testing-debug.md 의
# `grep -rl ENABLE_COVERAGE ...` 나 architecture.md 의 `grep -n 'struct.*Data'` 은
# 사용자가 실행하는 운영 지시이지 자가검사 게이트가 아니므로 대상이 아니다.
# 아래 파일들의 grep 은 정의상 전부 탐지 패턴이다.
DETECTION_PATTERN_SCOPE = (
    "agent_docs/invariants.md",
    "agent_docs/anti-patterns.md",
    "CLAUDE.md",
    "AGENTS.md",
    ".claude/rules/",
)

# D006: 산문·코드스팬에 등장하는 저장소 경로. 최상위 패키지 디렉토리로 시작하는
# 것만 검사해 오탐을 줄인다 (`types/types.hpp` 같은 패키지 상대 include 축약형은
# 대상 아님). 생성 헤더(`rtc_msgs/msg/*.hpp`)와 생략 표기(`...`)는 제외한다.
REPO_PATH_RE = re.compile(
    r"(?<![\w./-])((?:[\w-]+/){1,6}[\w.-]+\.(?:cpp|hpp|h|py|sh|md|yaml|yml|txt|xml))(?![\w/])"
)

LINK_RE = re.compile(r"(?<!!)\[(?P<text>[^\]]*)\]\((?P<target>[^)\s]+)(?:\s+\"[^\"]*\")?\)")
LINE_ANCHOR_RE = re.compile(r"#L\d+(?:-L?\d+)?$")
# D003 은 "정책이 위치를 명명하는 것" 과 "문서가 특정 private 파일에 의존하는 것" 을
# 구분해야 한다. CLAUDE.md / handoff.md 는 private plan 의 보관 위치를 규정하는
# 문서이므로 `~/.claude/plans/<slug>.md` 같은 placeholder 를 반드시 쓴다 — 정당하다.
# 결함은 커밋된 문서가 **구체 파일명** 을 결정의 출처로 가리킬 때다 (그 파일은
# agent-local 이고 완료 시 삭제되므로 다른 누구도 따라갈 수 없다).
# placeholder 는 `<` / `*` 를 포함해 아래 문자클래스에 걸리지 않는다.
CONCRETE_PLAN_RE = re.compile(r"~/\.claude/plans/[\w.-]+\.md")
# /home/runner/ 는 GitHub Actions 가 정의하는 러너 경로라 machine-local 이 아니다.
ABS_HOME_RE = re.compile(r"/home/(?!runner/)[\w.-]+/[\w./-]+")
# YAML/셸 주석 안에서 repo 경로처럼 보이는 토큰. 확장자를 요구해 산문 오탐을 줄인다.
BARE_PATH_RE = re.compile(
    r"(?<![\w./-])((?:[\w-]+/)+[\w.-]+\.(?:md|sh|py|yml|yaml|repos))(?![\w/])"
)
HEADING_RE = re.compile(r"^(#{1,6})\s+(.*?)\s*#*\s*$")
FENCE_RE = re.compile(r"^\s*(```|~~~)")


@dataclass(frozen=True)
class Finding:
    code: str
    path: str
    line: int
    message: str


def repo_root() -> Path:
    out = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        capture_output=True,
        text=True,
        check=True,
    )
    return Path(out.stdout.strip())


def iter_files(root: Path) -> list[Path]:
    """검사 대상: 저장소 전체의 *.md + .github 의 workflow/action 정의."""
    found: list[Path] = []
    for path in root.rglob("*"):
        if any(part in SKIP_DIRS for part in path.relative_to(root).parts):
            continue
        if not path.is_file():
            continue
        rel = path.relative_to(root).as_posix()
        if path.suffix == ".md" or path.suffix in (".yml", ".yaml") and rel.startswith(".github/"):
            found.append(path)
    return sorted(found)


def slugify(heading: str) -> str:
    """GitHub 의 heading → 앵커 슬러그 규칙 근사.

    소문자화 → 인라인 마크업 제거 → 공백을 하이픈으로 → 구두점 제거.
    한글 등 non-ASCII 는 보존한다 (GitHub 도 보존).

    `_` 는 emphasis 문자지만 여기서 제거하면 안 된다 — heading 에 등장하는 `_` 는
    거의 전부 식별자의 일부이고 (`ros2_control`, `rtc_base`), GitHub 도 슬러그에서
    보존한다. 제거하면 `ros2_control` 이 `ros2control` 이 돼 성한 앵커를
    깨진 것으로 오보한다.
    """
    text = re.sub(r"`([^`]*)`", r"\1", heading)
    text = LINK_RE.sub(lambda m: m.group("text"), text)
    text = re.sub(r"[*~]", "", text)
    text = text.strip().lower()
    text = re.sub(r"[^\w\s-]", "", text, flags=re.UNICODE)
    return re.sub(r"\s+", "-", text)


def headings_of(path: Path) -> set[str]:
    slugs: set[str] = set()
    in_fence = False
    for raw in path.read_text(encoding="utf-8", errors="replace").splitlines():
        if FENCE_RE.match(raw):
            in_fence = not in_fence
            continue
        if in_fence:
            continue
        m = HEADING_RE.match(raw)
        if m:
            slugs.add(slugify(m.group(2)))
    return slugs


def code_spans(line: str) -> list[str]:
    return re.findall(r"`([^`]+)`", line)


def mask_code_spans(line: str) -> str:
    """인라인 코드 스팬을 같은 길이의 공백으로 치환 (컬럼 오프셋 보존).

    링크 검사가 코드 예시를 링크로 오독하는 것을 막는다 — `operator[](name)` 은
    C++ 시그니처지 `name` 파일로 가는 링크가 아니다.
    """
    return re.sub(r"`[^`]+`", lambda m: " " * len(m.group(0)), line)


def check_file(path: Path, root: Path, heading_cache: dict[Path, set[str]]) -> list[Finding]:
    rel = path.relative_to(root).as_posix()
    findings: list[Finding] = []
    text = path.read_text(encoding="utf-8", errors="replace")
    is_md = path.suffix == ".md"
    in_fence = False

    for lineno, raw in enumerate(text.splitlines(), start=1):
        if is_md and FENCE_RE.match(raw):
            in_fence = not in_fence
            continue

        # --- D003: private / machine-local 경로 -----------------------------
        # 커밋된 문서가 agent-local plan 파일에 의존하면 그 문서는 다른 누구에게도
        # 해석 불가능하다. CLAUDE.md §6.6 / handoff.md 가 private plan 을
        # "완료 시 삭제" 로 규정하므로 애초에 영속 참조 대상이 아니다.
        for hit in CONCRETE_PLAN_RE.findall(raw):
            findings.append(
                Finding(
                    "D003",
                    rel,
                    lineno,
                    f"구체 private plan 파일 `{hit}` 참조 — 그 파일은 agent-local 이고 완료 시 "
                    "삭제되므로 결정의 출처가 될 수 없다. 근거를 이 문서나 issue 로 옮길 것",
                )
            )
        for hit in ABS_HOME_RE.findall(raw):
            findings.append(
                Finding(
                    "D003",
                    rel,
                    lineno,
                    f"machine-local 절대경로 `{hit}` — 저장소 상대경로나 placeholder 로 바꿀 것",
                )
            )

        if is_md and not in_fence:
            for m in LINK_RE.finditer(mask_code_spans(raw)):
                findings.extend(_check_link(m, path, root, rel, lineno, heading_cache))

            # --- D004: 문서에 박힌 raw 탐지 정규식 -------------------------
            # 근본 원인은 구조적이다 — 마크다운 표 셀은 이스케이프 없이 `|` 를
            # 담을 수 없어 ERE 가 반드시 깨진다. 그러나 표 밖(`- **탐지**:` 불릿)
            # 에서도 divergent copy 가 자란다: 같은 규칙의 패턴이 invariants.md 와
            # anti-patterns.md 에 서로 다르게 존재한다. 그래서 위치를 가리지 않고
            # 탐지 패턴 자체를 문서에서 추방한다 (issue #213 결정 3).
            if rel.startswith(DETECTION_PATTERN_SCOPE):
                for span in code_spans(raw):
                    if not re.search(r"\bgrep\b|\brg\b", span):
                        continue
                    # 인용부호로 감싼 패턴 인자가 있어야 '탐지 패턴' 이다. 이 게이트가
                    # 정당한 산문을 살려 준다 — `ps -eLf | grep <process> | wc -l`
                    # (셸 파이프라인), `grep -c <new_pattern>` (플레이스홀더),
                    # `grep -l registerNodeType <dir>` (리터럴 검색) 은 자가검사
                    # 게이트가 아니므로 hook 으로 옮길 대상이 아니다.
                    if not re.search(r"""(['"]).*\1""", span):
                        continue
                    findings.append(
                        Finding(
                            "D004",
                            rel,
                            lineno,
                            f"문서에 박힌 탐지 패턴 ({_broken_ere_reason(span)}): `{span}` — "
                            "정규식 SSoT 는 .claude/hooks/verify-changes.sh 다. "
                            "문서는 hook 의 검사명으로 인용만 할 것",
                        )
                    )

            # --- D006: 산문/코드스팬의 저장소 경로가 실재하는가 --------------
            # architecture.md 의 `integrated_bringup/support/owned_topics.cpp`
            # (실제는 `src/support/...`) 처럼, 링크가 아니라 평문으로 적힌 경로는
            # 어떤 링크 체커도 잡지 못한 채 조용히 stale 해진다.
            for cand in REPO_PATH_RE.findall(raw):
                if "..." in cand or not _is_repo_path(cand, root):
                    continue
                if not _resolves(cand, root):
                    findings.append(
                        Finding(
                            "D006",
                            rel,
                            lineno,
                            f"존재하지 않는 저장소 경로 `{cand}`",
                        )
                    )

        # --- D001(보조): YAML/주석 안의 bare repo 경로 --------------------
        if not is_md:
            stripped = raw.strip()
            if stripped.startswith("#"):
                for cand in BARE_PATH_RE.findall(raw):
                    if not (root / cand).exists():
                        findings.append(
                            Finding(
                                "D001",
                                rel,
                                lineno,
                                f"존재하지 않는 저장소 경로 참조 `{cand}`",
                            )
                        )

    return findings


_TOP_LEVEL_DIRS: set[str] | None = None
_TRACKED_BY_BASENAME: dict[str, list[str]] | None = None
# rosidl 이 빌드 시 생성하는 헤더 — 트리에 없는 게 정상이다.
GENERATED_RE = re.compile(r"^(rtc_msgs|shape_estimation_msgs)/msg/.*\.(hpp|h)$")


def _tracked_index(root: Path) -> dict[str, list[str]]:
    global _TRACKED_BY_BASENAME
    if _TRACKED_BY_BASENAME is None:
        out = subprocess.run(
            ["git", "-C", str(root), "ls-files"],
            capture_output=True,
            text=True,
            check=True,
        )
        index: dict[str, list[str]] = {}
        for line in out.stdout.splitlines():
            index.setdefault(line.rsplit("/", 1)[-1], []).append(line)
        _TRACKED_BY_BASENAME = index
    return _TRACKED_BY_BASENAME


def _is_repo_path(cand: str, root: Path) -> bool:
    """평문 경로가 '검사할 가치가 있는 저장소 경로' 인지 판정한다.

    첫 세그먼트가 실재하는 최상위 디렉토리여야 한다 — `types/types.hpp` 같은 패키지
    내부 상대 표기를 걸러낸다.
    """
    global _TOP_LEVEL_DIRS
    if _TOP_LEVEL_DIRS is None:
        _TOP_LEVEL_DIRS = {p.name for p in root.iterdir() if p.is_dir()}
    if GENERATED_RE.match(cand):
        return False
    return cand.split("/", 1)[0] in _TOP_LEVEL_DIRS


def _resolves(cand: str, root: Path) -> bool:
    """저장소 루트 기준 실재하거나, 어떤 tracked 파일의 경로 접미사이면 성한 참조다.

    문서는 헤더를 **include 경로** 로 인용한다: `rtc_base/types/types.hpp` 는 실제로
    `rtc_base/include/rtc_base/types/types.hpp` 에 있고, 이는 정당한 표기다. 접미사
    매칭이 이런 축약형 전부를 해석해 준다.

    동시에 접미사 매칭은 진짜 결함을 놓치지 않는다 —
    `integrated_bringup/support/owned_topics.cpp` 는 실제 경로가
    `integrated_bringup/src/support/owned_topics.cpp` 라 그 접미사가 아니므로 걸린다.
    """
    if (root / cand).exists():
        return True
    basename = cand.rsplit("/", 1)[-1]
    suffix = "/" + cand
    return any(p.endswith(suffix) for p in _tracked_index(root).get(basename, ()))


def _broken_ere_reason(span: str) -> str | None:
    """표 셀 안에서 깨진 ERE 의 구체적 사유. 성한 패턴은 None."""
    if r"\|" in span:
        return r"`\|` 를 포함한다 (ERE 에서 리터럴 파이프 → 영원히 미매치)"
    if "\\\\" in span:
        return "백슬래시가 이중화돼 있다 (ERE syntax error → exit 2)"
    if re.search(r"\((?:\||[^)]*\|\|)", span):
        return "빈 alternation 분기를 갖는다 (ugrep 은 hard error)"
    return "표 셀에 있다 (파이프 이스케이프 없이는 유지 불가)"


def _check_link(
    m: re.Match[str],
    path: Path,
    root: Path,
    rel: str,
    lineno: int,
    heading_cache: dict[Path, set[str]],
) -> list[Finding]:
    target = m.group("target")
    if re.match(r"^(https?:|mailto:|#)", target):
        return []

    file_part, _, fragment = target.partition("#")
    findings: list[Finding] = []

    # --- D002: 라인 앵커 전면 금지 -------------------------------------
    # issue #213 코멘트 2 (추가 D): 이번 감사 라운드에서만 인용 라인이 4곳
    # 어긋났고 그중 하나는 결론의 방향까지 뒤집혔다. 사람이든 에이전트든 라인
    # 번호로 코드를 인용하면 drift 한다 — 심볼/함수명으로 인용할 것.
    if fragment and LINE_ANCHOR_RE.match("#" + fragment):
        findings.append(
            Finding(
                "D002",
                rel,
                lineno,
                f"라인 앵커 `#{fragment}` (→ {file_part or rel}) — 편집마다 drift 한다. "
                "심볼/함수명으로 인용할 것",
            )
        )

    if not file_part:
        return findings

    resolved = (path.parent / file_part).resolve()
    try:
        resolved.relative_to(root.resolve())
    except ValueError:
        # 저장소 밖으로 나가는 상대링크. 커밋된 문서에서는 항상 결함이다 — 클론한
        # 사람의 디스크에 그 경로가 있을 이유가 없다 (관측 사례: docs 가 `../../.claude/
        # projects/<사용자명 박힌 경로>/memory/*.md` 를 가리킴).
        findings.append(
            Finding(
                "D003",
                rel,
                lineno,
                f"저장소 밖을 가리키는 링크 `{target}` — 클론한 트리에서는 해석 불가능하다",
            )
        )
        return findings

    if not resolved.exists():
        findings.append(
            Finding(
                "D001",
                rel,
                lineno,
                f"깨진 링크 `{target}` — 대상이 없다",
            )
        )
        return findings

    # --- D005: heading 앵커 -------------------------------------------
    if fragment and not LINE_ANCHOR_RE.match("#" + fragment) and resolved.suffix == ".md":
        if resolved not in heading_cache:
            heading_cache[resolved] = headings_of(resolved)
        want = unicodedata.normalize("NFC", fragment.lower())
        have = {unicodedata.normalize("NFC", s) for s in heading_cache[resolved]}
        if want not in have:
            findings.append(
                Finding(
                    "D005",
                    rel,
                    lineno,
                    f"heading 앵커 `#{fragment}` 가 {file_part} 에 없다",
                )
            )

    return findings


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--format",
        choices=("text", "github"),
        default="text",
        help="github 은 GitHub Actions 어노테이션을 함께 출력한다",
    )
    args = parser.parse_args()

    root = repo_root()
    heading_cache: dict[Path, set[str]] = {}
    findings: list[Finding] = []
    for path in iter_files(root):
        findings.extend(check_file(path, root, heading_cache))

    if not findings:
        print("validate_docs: 결함 없음")
        return 0

    by_code: dict[str, list[Finding]] = {}
    for f in findings:
        by_code.setdefault(f.code, []).append(f)

    for code in sorted(by_code):
        group = by_code[code]
        print(f"\n=== {code} — {len(group)}건 ===")
        for f in group:
            print(f"  {f.path}:{f.line}: {f.message}")
            if args.format == "github":
                print(f"::error file={f.path},line={f.line}::{f.code} {f.message}")

    print(f"\nvalidate_docs: 총 {len(findings)}건 실패")
    return 1


if __name__ == "__main__":
    sys.exit(main())
