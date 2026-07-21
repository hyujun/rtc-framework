#!/usr/bin/env python3
"""Documentation-corpus validator (issue #213).

The agent-facing documentation in this repository carries executable claims:
relative links, code-location citations, and the invariant *detection patterns*
that CLAUDE.md tells an agent to run as a pre-commit self-check.  None of that
is covered by ``colcon test``, and the CI workflow explicitly ``paths-ignore``s
``agent_docs/**``.  When a detection pattern rots, it does not fail loudly --
``grep`` exits 1 with no output, which reads exactly like "no violations".
This validator turns that silence into a red build.

Checks
------
D1  link / file-reference target exists (markdown links + path-shaped inline
    code spans).  Header include shorthand (``rtc_base/types/types.hpp`` for
    ``rtc_base/include/rtc_base/types/types.hpp``) resolves via path-component
    suffix matching, so a genuinely wrong path such as
    ``integrated_bringup/support/owned_topics.cpp`` (missing ``src/``) is still
    reported.
D2  in-repo anchor exists, using GitHub's heading-slug algorithm (see
    :func:`slugify` -- ``## Build & Test`` really is ``#build--test``).
D3  no link escapes the repository, and no ``/home/<user>/`` absolute path
    appears anywhere in the corpus (including shell snippets, which get copied).
D4  no ``file.cpp:123`` / ``#L123`` line-anchor citations in the agent corpus.
    Line numbers drift on every edit; cite a symbol instead.
D7  ``detect`` fenced blocks: the pattern is linted for the escaping mistakes
    that make a grep silently match nothing, then executed against a required
    ``# probe:`` line it must match (and any ``# antiprobe:`` it must not), so
    a pattern that compiles but can no longer fire still fails.  An optional
    ``# exemplar:`` additionally asserts the state of the tree today.
D8  no detection pattern outside a detect block.  A markdown table cell cannot
    hold an unescaped ``|``, so a regex parked in one gets escaped into
    something inert; the table is the root cause, not the individual typos.

Both a static lint *and* an execution check are required, and neither subsumes
the other.  A doubled backslash (``\\\\b``) makes grep exit 2 -- loud.  But
``\\|`` inside an ERE is *valid syntax* for a literal pipe: exit 1, no output,
indistinguishable from a clean tree.  Execution alone would have graded this
corpus healthy.

Self-verification
-----------------
``--self-test`` runs the checker against fixtures that encode the silent-pass
inputs a previous attempt at this validator got wrong (option clusters such as
``grep -rn --include='*.cpp' -E 'a\\|b'`` misgraded as BRE, the pattern
argument shadowed by ``--include``), plus GitHub slug conformance cases.  A
checker for silent failures cannot be trusted because it is green on the
repository; it has to be green on inputs whose answer is known.

Usage
-----
    validate_docs.py                 # full corpus scan (CI)
    validate_docs.py --files a.md b  # changed-file scope (Stop hook)
    validate_docs.py --self-test     # fixture suite for the checker itself
"""

from __future__ import annotations

import argparse
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path

# Extensions worth resolving when a token looks like a repository path.
PATH_EXTS = (
    ".md",
    ".cpp",
    ".hpp",
    ".h",
    ".cc",
    ".py",
    ".sh",
    ".yaml",
    ".yml",
    ".xml",
    ".txt",
    ".json",
    ".repos",
    ".csv",
)

# Generated / external trees a documentation path may legitimately mention
# without the file being tracked in git.
IGNORED_PATH_PREFIXES = (
    "build/",
    "install/",
    "log/",
    "opt/",
    "usr/",
    "etc/",
    "var/",
    "proc/",
    "sys/",
    "tmp/",
    "dev/",
    "node_modules/",
    "site-packages/",
    ".vscode/",  # editor config the docs tell a reader to create
)

# grep options that consume the following argument.  Everything else is either
# a flag or an operand; getting this wrong is how a previous validator let
# `--include='*.cpp'` masquerade as the pattern.
GREP_SHORT_WITH_VALUE = set("efmABCDd")
GREP_LONG_WITH_VALUE = {
    "--regexp",
    "--file",
    "--max-count",
    "--include",
    "--exclude",
    "--exclude-dir",
    "--exclude-from",
    "--include-dir",
    "--after-context",
    "--before-context",
    "--context",
    "--color",
    "--colour",
    "--binary-files",
    "--devices",
    "--directories",
    "--label",
}

# GitHub heading slugs drop punctuation *except* `-` and `_`, and do not
# collapse the whitespace that removal leaves behind.  ASCII ranges: C0/C1
# controls, 0x21-0x2C (`!` through `,`), `.`, `/`, 0x3A-0x40 (`:` through `@`),
# 0x5B-0x5E (`[` through `^`), backtick, 0x7B-0x7E (`{` through `~`).
#
# The Unicode ranges are not optional garnish.  This repository's headings use
# an em dash as a separator ("3. Launch 디버거 — 노드 직접 실행"), and GitHub
# strips U+2014 along with the rest of General Punctuation, leaving the double
# hyphen that the working in-repo links already use.  An ASCII-only strip set
# rejects those live anchors while happily accepting ones that 404 -- the
# precise inversion that sank the first attempt at this validator.
SLUG_STRIP_RE = re.compile(
    "["
    r"\x00-\x1f\x7f-\x9f"  # C0 / C1 controls
    r"!-,./:-@\[-^`{-~"  # ASCII punctuation, minus `-` and `_`
    r"\xa0-\xbf\xd7\xf7"  # Latin-1 punctuation and math signs
    "\u1680\u2000-\u206f"  # Ogham space, General Punctuation (em dash!)
    "\u2e00-\u2e7f"  # Supplemental Punctuation
    "\u3000-\u303f"  # CJK Symbols and Punctuation
    "\ufe10-\ufe6f"  # vertical forms, small form variants
    "\uff01-\uff20\uff3b-\uff40\uff5b-\uff65"  # fullwidth forms
    "\ufeff"  # BOM
    "]"
)

MD_LINK_RE = re.compile(r"(?<!\!)\[(?:[^\]\\]|\\.)*\]\(([^)\s]+)(?:\s+\"[^\"]*\")?\)")
CODE_SPAN_RE = re.compile(r"`([^`\n]+)`")
HEADING_RE = re.compile(r"^(#{1,6})\s+(.*?)\s*#*\s*$")
HTML_ANCHOR_RE = re.compile(r"<a\s+(?:name|id)=[\"']([^\"']+)[\"']")
FENCE_RE = re.compile(r"^(\s*)(`{3,}|~{3,})(.*)$")
HOME_PATH_RE = re.compile(r"/home/[A-Za-z0-9_.-]+/")
LINE_ANCHOR_RE = re.compile(r"[\w./-]+\.(?:cpp|hpp|h|cc|py|sh|md|yaml|yml|xml|txt):\d+")
L_ANCHOR_RE = re.compile(r"#L\d+")
INLINE_GREP_RE = re.compile(r"(?:^|\|\s*)r?g?grep\s+-{1,2}\w")
BARE_PATH_RE = re.compile(r"(?<![\w./-])((?:[\w.-]+/)+[\w.-]+\.[A-Za-z0-9]+)")

# Corpus whose code citations must be symbol-based (D4).
SYMBOL_CITATION_DIRS = ("agent_docs/", ".claude/")


@dataclass(frozen=True)
class Finding:
    path: str
    line: int
    code: str
    message: str

    def __str__(self) -> str:
        return f"{self.path}:{self.line}: [{self.code}] {self.message}"


# --------------------------------------------------------------------------
# GitHub heading slugs
# --------------------------------------------------------------------------


def slugify(text: str) -> str:
    """Reproduce github-slugger for a single heading.

    Spaces become hyphens *after* punctuation removal, and removal does not
    collapse the gap it leaves.  ``## Build & Test`` therefore anchors at
    ``#build--test``, and a validator that folds whitespace first rejects the
    anchor that actually works while accepting the one that 404s.
    """
    # GitHub renders the heading before slugging it: link syntax contributes
    # its text, images contribute their alt text, HTML comments vanish.
    #
    # Code spans are stashed first because their contents are literal text, not
    # markup.  A heading like "(sidecar `<name>.closure.yaml`)" anchors at
    # ...-nameclosureyaml; treating `<name>` as an HTML tag deletes it and
    # invents an anchor nobody can link to.
    spans: list[str] = []

    def stash(m: re.Match) -> str:
        spans.append(m.group(1))
        return f"\x00{len(spans) - 1}\x00"

    text = CODE_SPAN_RE.sub(stash, text)
    text = re.sub(r"<!--.*?-->", "", text, flags=re.S)
    text = re.sub(r"!\[([^\]]*)\]\([^)]*\)", r"\1", text)
    text = re.sub(r"\[([^\]]*)\]\([^)]*\)", r"\1", text)
    text = re.sub(r"<[^>]+>", "", text)
    text = re.sub(r"\x00(\d+)\x00", lambda m: spans[int(m.group(1))], text)
    slug = text.strip().lower()
    slug = SLUG_STRIP_RE.sub("", slug)
    return slug.replace(" ", "-")


def anchors_of(text: str) -> set[str]:
    """Every anchor a rendered markdown document exposes."""
    anchors: set[str] = set()
    counts: dict[str, int] = {}
    for raw, in_fence in iter_lines_with_fence_state(text):
        if in_fence:
            continue
        m = HEADING_RE.match(raw)
        if m:
            base = slugify(m.group(2))
            if not base:
                continue
            n = counts.get(base, 0)
            counts[base] = n + 1
            anchors.add(base if n == 0 else f"{base}-{n}")
        for a in HTML_ANCHOR_RE.findall(raw):
            anchors.add(a)
    return anchors


def iter_lines_with_fence_state(text: str):
    """Yield ``(line, inside_fenced_block)`` pairs."""
    fence: str | None = None
    for raw in text.splitlines():
        m = FENCE_RE.match(raw)
        if m:
            marker = m.group(2)
            if fence is None:
                fence = marker[0] * 3
                yield raw, True
                continue
            if marker[0] * 3 == fence:
                fence = None
                yield raw, True
                continue
        yield raw, fence is not None


# --------------------------------------------------------------------------
# grep command analysis (D7 static lint)
# --------------------------------------------------------------------------


def parse_grep(cmdline: str) -> tuple[str, str | None, str | None]:
    """Return ``(flavor, pattern, error)`` for a grep invocation.

    ``flavor`` is one of ``ere`` / ``bre`` / ``pcre`` / ``fixed``.  Options are
    parsed properly rather than by position: the pattern is the first *operand*
    (or the value of ``-e``), so ``--include='*.cpp'`` can never be mistaken
    for it, and ``-E`` is honoured wherever it appears -- including inside a
    cluster such as ``-rnE`` or separated as ``-r -n -E``.
    """
    try:
        tokens = _shlex_split(cmdline)
    except ValueError as exc:  # unbalanced quotes
        return "bre", None, f"unparseable command line ({exc})"
    if not tokens or "grep" not in tokens[0]:
        return "bre", None, "detect block must be a grep invocation"

    flavor = "bre"
    pattern: str | None = None
    operands: list[str] = []
    i = 1
    end_of_options = False
    while i < len(tokens):
        tok = tokens[i]
        if end_of_options or tok == "-" or not tok.startswith("-"):
            operands.append(tok)
            i += 1
            continue
        if tok == "--":
            end_of_options = True
            i += 1
            continue
        if tok.startswith("--"):
            name, sep, value = tok.partition("=")
            if name in ("--extended-regexp",):
                flavor = "ere"
            elif name in ("--perl-regexp",):
                flavor = "pcre"
            elif name in ("--fixed-strings",):
                flavor = "fixed"
            elif name in ("--basic-regexp",):
                flavor = "bre"
            if name in ("--regexp", "--file"):
                got = value if sep else (tokens[i + 1] if i + 1 < len(tokens) else "")
                if name == "--regexp" and pattern is None:
                    pattern = got
            if name in GREP_LONG_WITH_VALUE and not sep:
                i += 1
            i += 1
            continue
        # Short option cluster.
        j = 1
        consumed_next = False
        while j < len(tok):
            ch = tok[j]
            if ch == "E":
                flavor = "ere"
            elif ch == "P":
                flavor = "pcre"
            elif ch == "F":
                flavor = "fixed"
            elif ch == "G":
                flavor = "bre"
            if ch in GREP_SHORT_WITH_VALUE:
                rest = tok[j + 1 :]
                if rest:
                    value = rest
                else:
                    value = tokens[i + 1] if i + 1 < len(tokens) else ""
                    consumed_next = True
                if ch == "e" and pattern is None:
                    pattern = value
                break
            j += 1
        i += 2 if consumed_next else 1
    if pattern is None and operands:
        pattern = operands[0]
    if pattern is None:
        return flavor, None, "no pattern found in detect command"
    return flavor, pattern, None


def _shlex_split(cmdline: str) -> list[str]:
    import shlex

    return shlex.split(cmdline)


def lint_pattern(flavor: str, pattern: str) -> list[str]:
    """Escaping mistakes that make a pattern match nothing, or refuse to run."""
    problems: list[str] = []
    if flavor in ("fixed", "pcre"):
        return problems
    # Blank out escaped characters before looking for structural mistakes:
    # `malloc\(|` contains the byte pair "(|" but that paren is a literal, not
    # an empty group -- flagging it would condemn a correct pattern.
    bare = re.sub(r"\\.", "\x00", pattern)
    if re.search(r"\(\||\|\)|\|\|", bare):
        problems.append(
            "empty alternation branch -- GNU grep tolerates it, the sandbox's "
            "ugrep rejects it outright ('empty (sub)expression'); write "
            r"'(_for|_until)?' instead"
        )
    if flavor == "ere":
        if r"\|" in pattern:
            problems.append(
                r"'\|' in an ERE is a *literal pipe*, not alternation -- the "
                "pattern is valid syntax that can never match (exit 1, no "
                "output, reads as 'no violations'); use '|'"
            )
        if re.search(r"\\\\[A-Za-z(){}.[\]]", pattern):
            problems.append(
                r"doubled backslash (e.g. '\\b', '\\(') -- an artefact of "
                "escaping for a markdown table cell; grep exits 2 on it"
            )
    return problems


def compile_pattern(flavor: str, pattern: str) -> tuple[re.Pattern | None, str | None]:
    """Compile an ERE/BRE pattern with Python's engine for execution."""
    if flavor == "fixed":
        return re.compile(re.escape(pattern)), None
    src = pattern
    if flavor == "bre":
        # BRE: `\(`/`\|` are metacharacters, bare `(`/`|` are literals.
        placeholder = "\x00"
        src = src.replace("(", placeholder + "(").replace(")", placeholder + ")")
        src = src.replace("|", placeholder + "|")
        src = re.sub(r"\\\x00([(|)])", r"\1", src)
        src = src.replace(placeholder, "\\")
    try:
        return re.compile(src), None
    except re.error as exc:
        return None, str(exc)


# --------------------------------------------------------------------------
# Repository index
# --------------------------------------------------------------------------


class Repo:
    def __init__(self, root: Path):
        self.root = root
        out = subprocess.run(
            ["git", "-C", str(root), "-c", "core.quotePath=false", "ls-files", "-z"],
            capture_output=True,
            text=True,
            check=True,
        ).stdout
        self.files = [f for f in out.split("\0") if f]
        self.fileset = set(self.files)
        # Path-component suffix index: an include-style shorthand such as
        # `rtc_base/types/types.hpp` must resolve to the real
        # `rtc_base/include/rtc_base/types/types.hpp` without also letting a
        # wrong path like `integrated_bringup/support/owned_topics.cpp` pass
        # (its components are not a suffix of the real ones).
        self.suffixes: set[str] = set()
        for f in self.files:
            parts = f.split("/")
            for k in range(1, len(parts) + 1):
                self.suffixes.add("/".join(parts[-k:]))
        # First path component of everything the repository actually contains.
        # A citation whose leading component is not in here names something
        # outside this tree -- a system header (`arpa/inet.h`), an external ROS
        # package (`ur_robot_driver/...`), a runtime output directory -- and
        # cannot be resolved or usefully reported.
        self.toplevel = {f.split("/")[0] for f in self.files}
        self.toplevel |= {p.name for p in self.root.iterdir()}
        self._anchor_cache: dict[str, set[str]] = {}

    def exists(self, rel: str) -> bool:
        if rel in self.fileset or (self.root / rel).exists():
            return True
        return rel in self.suffixes

    def resolve(self, token: str, from_file: str) -> bool:
        """Does ``token``, cited in ``from_file``, name something real?

        Documentation cites paths from several vantage points: repo-relative,
        relative to the citing file, relative to the owning package, and as an
        include-style shorthand.  All four are legitimate, so all four are
        tried before a citation is called broken.
        """
        if self.exists(token):
            return True
        here = Path(from_file).parent
        candidates = [(here / token).as_posix()]
        pkg = from_file.split("/")[0]
        if pkg != from_file:
            candidates.append(f"{pkg}/{token}")
        for cand in candidates:
            norm = normalise(cand)
            if norm and self.exists(norm):
                return True
        # ROS generates message/service headers into the build tree; they are
        # real at runtime and absent from the source index.
        return bool(re.match(r"^[\w]+/(msg|srv|action)/[\w]+\.(hpp|h|py)$", token))

    def anchors(self, rel: str) -> set[str]:
        if rel not in self._anchor_cache:
            p = self.root / rel
            try:
                self._anchor_cache[rel] = anchors_of(p.read_text(encoding="utf-8"))
            except OSError:
                self._anchor_cache[rel] = set()
        return self._anchor_cache[rel]

    def corpus(self) -> list[str]:
        keep = []
        for f in self.files:
            if (
                f.endswith(".md")
                or f.startswith(".github/workflows/")
                and f.endswith((".yml", ".yaml"))
            ):
                keep.append(f)
        return keep


# --------------------------------------------------------------------------
# Checks
# --------------------------------------------------------------------------


def check_file(repo: Repo, rel: str) -> list[Finding]:
    try:
        text = (repo.root / rel).read_text(encoding="utf-8")
    except OSError as exc:
        return [Finding(rel, 1, "D0", f"unreadable: {exc}")]
    if rel.endswith(".md"):
        return check_markdown(repo, rel, text)
    return check_workflow(repo, rel, text)


def check_markdown(repo: Repo, rel: str, text: str) -> list[Finding]:
    findings: list[Finding] = []
    base = Path(rel).parent
    in_symbol_corpus = rel.startswith(SYMBOL_CITATION_DIRS)

    for lineno, (raw, in_fence) in enumerate(iter_lines_with_fence_state(text), 1):
        # D3 absolute private paths: checked inside fences too -- a shell
        # snippet is the copy-paste surface that actually breaks for others.
        for m in HOME_PATH_RE.finditer(raw):
            findings.append(
                Finding(
                    rel,
                    lineno,
                    "D3",
                    f"machine-local absolute path '{m.group(0)}...' -- use a "
                    "repo-relative path or ${HOME}",
                )
            )
        if in_fence:
            continue

        # Link syntax is only link syntax outside a code span.  `operator[](name)`
        # is C++ being quoted, not a link to a file called "name"; masking the
        # spans first is what keeps this check from inventing work.
        for m in MD_LINK_RE.finditer(mask_code_spans(raw)):
            findings.extend(check_link(repo, rel, base, lineno, m.group(1)))

        for m in CODE_SPAN_RE.finditer(raw):
            findings.extend(check_path_token(repo, rel, lineno, m.group(1), "D1"))
            if in_symbol_corpus and is_parked_detection_pattern(m.group(1)):
                findings.append(
                    Finding(
                        rel,
                        lineno,
                        "D8",
                        "detection pattern outside a ```detect block -- a "
                        "markdown table cell cannot hold an unescaped '|', so "
                        "a pattern parked in one gets escaped into something "
                        "that silently matches nothing.  Move it to a fenced "
                        "detect block, where it is linted and executed.",
                    )
                )

        if in_symbol_corpus:
            for m in LINE_ANCHOR_RE.finditer(raw):
                findings.append(
                    Finding(
                        rel,
                        lineno,
                        "D4",
                        f"line-anchor citation '{m.group(0)}' -- line numbers "
                        "drift on every edit; cite the symbol / function name",
                    )
                )
            if L_ANCHOR_RE.search(raw):
                findings.append(
                    Finding(
                        rel,
                        lineno,
                        "D4",
                        "'#Lnnn' line anchor -- cite the symbol / function name",
                    )
                )

    findings.extend(check_detect_blocks(repo, rel, text))
    return findings


def mask_code_spans(raw: str) -> str:
    """Blank out inline code spans, preserving offsets."""
    return CODE_SPAN_RE.sub(lambda m: " " * len(m.group(0)), raw)


def is_parked_detection_pattern(span: str) -> bool:
    """Is this inline code span an invariant *detection* grep?

    Only extended/Perl regexes qualify.  Those are the ones that carry
    alternation, and alternation is what a markdown table cell cannot hold --
    the `|` has to be escaped, and `\\|` in an ERE is a literal pipe that
    matches nothing.  A plain lookup helper (`grep -rl ENABLE_COVERAGE ...`)
    has no such failure mode and must not be dragged into the detect-block
    regime; over-firing here would make the gate a nuisance that gets disabled.
    """
    span = span.strip()
    if not INLINE_GREP_RE.match(span):
        return False
    flavor, pattern, error = parse_grep(span)
    return error is None and pattern is not None and flavor in ("ere", "pcre")


def check_link(repo: Repo, rel: str, base: Path, lineno: int, target: str) -> list[Finding]:
    if re.match(r"^[a-zA-Z][a-zA-Z0-9+.-]*:", target) or target.startswith("//"):
        return []  # external scheme
    path_part, _, anchor = target.partition("#")
    findings: list[Finding] = []

    if not path_part:
        # Same-file anchor.
        if anchor and anchor not in repo.anchors(rel):
            findings.append(
                Finding(rel, lineno, "D2", f"anchor '#{anchor}' does not exist in this file")
            )
        return findings

    resolved = (base / path_part).as_posix()
    normalised = normalise(resolved)
    if normalised is None:
        return [
            Finding(
                rel,
                lineno,
                "D3",
                f"link '{target}' escapes the repository -- unresolvable in a fresh clone",
            )
        ]
    if not repo.exists(normalised) and not (repo.root / normalised).exists():
        return [Finding(rel, lineno, "D1", f"link target does not exist: '{path_part}'")]

    if anchor and normalised.endswith(".md") and anchor not in repo.anchors(normalised):
        findings.append(
            Finding(
                rel,
                lineno,
                "D2",
                f"anchor '#{anchor}' does not exist in {normalised}",
            )
        )
    return findings


def normalise(path: str) -> str | None:
    """Normalise a repo-relative path, or ``None`` if it escapes the root."""
    parts: list[str] = []
    for part in path.split("/"):
        if part in ("", "."):
            continue
        if part == "..":
            if not parts:
                return None
            parts.pop()
            continue
        parts.append(part)
    return "/".join(parts)


def check_path_token(repo: Repo, rel: str, lineno: int, token: str, code: str) -> list[Finding]:
    token = token.strip()
    if " " in token or "*" in token or "<" in token or ">" in token or "..." in token:
        return []
    if "/" not in token or not token.endswith(PATH_EXTS):
        return []
    if token.startswith(("~", "/", "http")) or token.startswith(IGNORED_PATH_PREFIXES):
        return []
    if "$" in token or "{" in token:
        return []
    head = normalise(token)
    if head is None:
        return []
    lead = head.split("/")[0]
    if lead not in repo.toplevel and not token.startswith((".", "..")):
        return []  # names something outside this repository
    if repo.resolve(token, rel):
        return []
    return [
        Finding(
            rel,
            lineno,
            code,
            f"path '{token}' does not exist (tried repo-, file-, and "
            "package-relative resolution plus include shorthand)",
        )
    ]


def check_workflow(repo: Repo, rel: str, text: str) -> list[Finding]:
    findings: list[Finding] = []
    for lineno, raw in enumerate(text.splitlines(), 1):
        for m in HOME_PATH_RE.finditer(raw):
            findings.append(
                Finding(rel, lineno, "D3", f"machine-local absolute path '{m.group(0)}...'")
            )
        # Only *documentation* references in comments are scanned.  A
        # workflow's executable lines name generated artefacts (coverage.xml),
        # action refs, and container paths, none of which exist in the source
        # tree; restricting to `.md` keeps the check to the thing that actually
        # rots -- a comment citing a design doc that has since been deleted.
        comment = raw.split("#", 1)[1] if "#" in raw else ""
        if not comment:
            continue
        for m in BARE_PATH_RE.finditer(comment):
            if m.group(1).endswith(".md"):
                findings.extend(check_path_token(repo, rel, lineno, m.group(1), "D1"))
    return findings


# --------------------------------------------------------------------------
# D7 -- detect blocks
# --------------------------------------------------------------------------

DETECT_OPEN_RE = re.compile(r"^\s*(`{3,})detect\b([^\n]*)$")


def parse_attrs(spec: str) -> dict[str, str]:
    return dict(re.findall(r"(\w+)=([^\s]+)", spec))


DIRECTIVE_RE = re.compile(r"^#\s*(probe|antiprobe|exemplar|expect)\s*:\s*(.*)$")


def split_directives(body: str) -> tuple[str, dict[str, list[str]]]:
    """Separate ``# probe:``-style directive lines from the grep command."""
    command: list[str] = []
    directives: dict[str, list[str]] = {}
    for line in body.splitlines():
        m = DIRECTIVE_RE.match(line.strip())
        if m:
            directives.setdefault(m.group(1), []).append(m.group(2))
        elif line.strip():
            command.append(line)
    return "\n".join(command).strip(), directives


def check_detect_blocks(repo: Repo, rel: str, text: str) -> list[Finding]:
    findings: list[Finding] = []
    lines = text.splitlines()
    i = 0
    while i < len(lines):
        m = DETECT_OPEN_RE.match(lines[i])
        if not m:
            i += 1
            continue
        fence, spec = m.group(1), m.group(2)
        attrs = parse_attrs(spec)
        start = i + 1
        j = start
        while j < len(lines) and not lines[j].strip().startswith(fence):
            j += 1
        body = "\n".join(lines[start:j]).strip()
        findings.extend(check_detect_block(repo, rel, start + 1, attrs, body))
        i = j + 1
    return findings


def check_detect_block(
    repo: Repo, rel: str, lineno: int, attrs: dict[str, str], body: str
) -> list[Finding]:
    findings: list[Finding] = []
    rule = attrs.get("id", "?")
    if "id" not in attrs:
        findings.append(Finding(rel, lineno, "D7", "detect block has no id= attribute"))
    body, directives = split_directives(body)
    flavor, pattern, error = parse_grep(body)
    if error or pattern is None:
        findings.append(Finding(rel, lineno, "D7", f"{rule}: {error}"))
        return findings

    for problem in lint_pattern(flavor, pattern):
        findings.append(Finding(rel, lineno, "D7", f"{rule}: {problem}"))

    compiled, cerror = compile_pattern(flavor, pattern)
    if compiled is None:
        findings.append(Finding(rel, lineno, "D7", f"{rule}: pattern does not compile ({cerror})"))
        return findings

    # A probe is a line the pattern MUST match: proof that the regex can still
    # fire at all.  It is required, and deliberately not sourced from the
    # repository -- a rule with zero current violations (RT-5, RT-6) would
    # otherwise be unverifiable, which is the exact state this whole issue is
    # about.  Exemplars are the weaker, optional check: they say something
    # about the tree today, and go stale when the tree changes for good reasons.
    probes = directives.get("probe", [])
    if not probes:
        findings.append(
            Finding(
                rel,
                lineno,
                "D7",
                f"{rule}: no '# probe:' line -- a pattern nobody ever ran "
                "against a known-matching input cannot be shown to fire",
            )
        )
    for probe in probes:
        if not compiled.search(probe):
            findings.append(
                Finding(
                    rel,
                    lineno,
                    "D7",
                    f"{rule}: pattern does not match its own probe {probe!r} -- "
                    "it would report 'no violations' on a genuine violation",
                )
            )
    for anti in directives.get("antiprobe", []):
        if compiled.search(anti):
            findings.append(
                Finding(
                    rel,
                    lineno,
                    "D7",
                    f"{rule}: pattern matches antiprobe {anti!r} -- too broad, "
                    "it will flag legitimate code and get ignored",
                )
            )

    for exemplar in directives.get("exemplar", []):
        expect = (directives.get("expect") or ["match"])[0]
        target = repo.root / exemplar
        if not target.exists():
            findings.append(
                Finding(rel, lineno, "D7", f"{rule}: exemplar '{exemplar}' does not exist")
            )
            continue
        hits = count_hits(target, compiled)
        if expect == "match" and hits == 0:
            findings.append(
                Finding(
                    rel,
                    lineno,
                    "D7",
                    f"{rule}: no match in exemplar '{exemplar}' -- either the "
                    "pattern rotted or the citation is stale",
                )
            )
        elif expect == "none" and hits > 0:
            findings.append(
                Finding(
                    rel, lineno, "D7", f"{rule}: expected no match in '{exemplar}', found {hits}"
                )
            )
    return findings


def count_hits(target: Path, compiled: re.Pattern) -> int:
    paths = [target] if target.is_file() else sorted(p for p in target.rglob("*") if p.is_file())
    total = 0
    for p in paths:
        try:
            content = p.read_text(encoding="utf-8", errors="replace")
        except OSError:
            continue
        total += sum(1 for line in content.splitlines() if compiled.search(line))
    return total


# --------------------------------------------------------------------------
# Self-test (B-2: a silent-pass detector must be proven on silent-pass input)
# --------------------------------------------------------------------------

GREP_CASES: list[tuple[str, str, str, bool]] = [
    # (command, expected flavor, expected pattern, expected to be flagged)
    ("grep -nE 'a\\|b' f.cpp", "ere", "a\\|b", True),
    ("grep -n -E 'a\\|b' f.cpp", "ere", "a\\|b", True),
    ("grep -rn --include='*.cpp' -E 'a\\|b' .", "ere", "a\\|b", True),
    ("grep -rnE --include='*.cpp' 'a\\|b' .", "ere", "a\\|b", True),
    ("grep --include='*.cpp' -rnE 'a\\|b' .", "ere", "a\\|b", True),
    ("grep -E -e 'a\\|b' f.cpp", "ere", "a\\|b", True),
    ("grep --extended-regexp 'a\\|b' f.cpp", "ere", "a\\|b", True),
    ("grep -niE '(\\\\bnew |x)' f.cpp", "ere", "(\\\\bnew |x)", True),
    ("grep -nE '\\.wait(|_for|_until)\\(' f.cpp", "ere", "\\.wait(|_for|_until)\\(", True),
    ("grep -nE '\\.wait(_for|_until)?\\(' f.cpp", "ere", "\\.wait(_for|_until)?\\(", False),
    # An escaped paren directly before an alternation is not an empty group.
    # Flagging it would reject the corrected RT-1 and RT-10 patterns.
    (
        "grep -nE '(\\bnew [A-Za-z_]|malloc\\(|\\.resize\\()' f.cpp",
        "ere",
        "(\\bnew [A-Za-z_]|malloc\\(|\\.resize\\()",
        False,
    ),
    (
        "grep -nE '(std::condition_variable|\\.notify_(one|all)\\(|\\.wait(_for|_until)?\\()' f.cpp",
        "ere",
        "(std::condition_variable|\\.notify_(one|all)\\(|\\.wait(_for|_until)?\\()",
        False,
    ),
    ("grep -n 'a\\|b' f.cpp", "bre", "a\\|b", False),
    ("grep -rnE '(ur5e|iiwa7)' rtc_base/", "ere", "(ur5e|iiwa7)", False),
    ("grep -c -m 3 -E 'foo|bar' f.cpp", "ere", "foo|bar", False),
]

SLUG_CASES: list[tuple[str, str]] = [
    ("Build & Test", "build--test"),
    ("설정 파일", "설정-파일"),
    ("Lifecycle 훅 ros2_control 연동", "lifecycle-훅-ros2_control-연동"),
    ("`RTC_REGISTER_CONTROLLER` 매크로", "rtc_register_controller-매크로"),
    ("9.1 colcon CWD (Hard rule)", "91-colcon-cwd-hard-rule"),
    ("[Link](http://x) Text", "link-text"),
    ("Trailing spaces   ", "trailing-spaces"),
    ("C++ / Python", "c--python"),
    # Regressions found by running this checker against the live corpus: an
    # em-dash separator, and a code span whose contents look like an HTML tag.
    # Both anchors work on GitHub; an approximation that drops them rejects
    # links that are fine and would have sent someone "fixing" them.
    ("3. Launch 디버거 — 노드 직접 실행", "3-launch-디버거--노드-직접-실행"),
    (
        "Extended-URDF closed-chain (sidecar `<name>.closure.yaml`)",
        "extended-urdf-closed-chain-sidecar-nameclosureyaml",
    ),
]

DUP_HEADINGS = ("# A\n\n# A\n\n# A\n", ["a", "a-1", "a-2"])

BRE_EXEC_CASES: list[tuple[str, str, str, bool]] = [
    # (flavor, pattern, subject, should match)
    # The escaping is inverted between the two flavours, which is the whole
    # reason a pattern can be moved between them and silently stop matching.
    ("bre", r"a\|b", "a", True),  # BRE: `\|` IS alternation
    ("bre", "a|b", "a", False),  # BRE: bare `|` is a literal
    ("bre", "a|b", "a|b", True),
    ("ere", r"a\|b", "a|b", True),  # ERE: `\|` is a literal pipe
    ("ere", r"a\|b", "a", False),  # ...so it can never match "a" alone
    ("ere", "(a|b)c", "bc", True),
]


PARKED_PATTERN_CASES: list[tuple[str, bool]] = [
    ("grep -nE 'RCLCPP_(INFO|WARN)\\(' <RT file>", True),
    ("grep -rniE '(ur5e|iiwa7)' rtc_*/", True),
    ("grep -rl ENABLE_COVERAGE */CMakeLists.txt", False),
    ("grep -n 'struct.*Data' <header>", False),
    ("colcon test --packages-select rtc_base", False),
    ("ament_add_gtest(... ENV ROS_DOMAIN_ID=<n>)", False),
]


def self_test() -> int:
    failures: list[str] = []

    for span, want in PARKED_PATTERN_CASES:
        got = is_parked_detection_pattern(span)
        if got != want:
            failures.append(f"is_parked_detection_pattern({span!r}) = {got}, want {want}")

    for cmd, want_flavor, want_pattern, want_flagged in GREP_CASES:
        flavor, pattern, error = parse_grep(cmd)
        if error:
            failures.append(f"parse_grep({cmd!r}) errored: {error}")
            continue
        if flavor != want_flavor:
            failures.append(f"parse_grep({cmd!r}) flavor={flavor!r} want {want_flavor!r}")
        if pattern != want_pattern:
            failures.append(f"parse_grep({cmd!r}) pattern={pattern!r} want {want_pattern!r}")
        flagged = bool(lint_pattern(flavor, pattern or ""))
        if flagged != want_flagged:
            failures.append(f"lint_pattern({cmd!r}) flagged={flagged} want {want_flagged}")

    if MD_LINK_RE.search(mask_code_spans("| `operator[](name)` | insertion order |")):
        failures.append("code-span masking failed: `operator[](name)` read as a link")
    if not MD_LINK_RE.search(mask_code_spans("see [architecture](agent_docs/architecture.md)")):
        failures.append("code-span masking destroyed a real link")

    for text, want in SLUG_CASES:
        got = slugify(text)
        if got != want:
            failures.append(f"slugify({text!r}) = {got!r}, want {want!r}")

    text, want_anchors = DUP_HEADINGS
    got_anchors = anchors_of(text)
    if got_anchors != set(want_anchors):
        failures.append(f"duplicate headings -> {sorted(got_anchors)}, want {want_anchors}")

    for flavor, pattern, subject, should_match in BRE_EXEC_CASES:
        compiled, err = compile_pattern(flavor, pattern)
        if compiled is None:
            failures.append(f"compile_pattern({flavor}, {pattern!r}) failed: {err}")
            continue
        got = bool(compiled.search(subject))
        if got != should_match:
            failures.append(
                f"compile_pattern({flavor}, {pattern!r}).search({subject!r}) = {got}, "
                f"want {should_match}"
            )

    # Path resolution: include shorthand must resolve, a wrong path must not.
    repo = Repo(repo_root())
    if not repo.exists("rtc_base/types/types.hpp"):
        failures.append("suffix match failed for include shorthand rtc_base/types/types.hpp")
    if repo.exists("integrated_bringup/support/owned_topics.cpp"):
        failures.append(
            "suffix match wrongly accepted integrated_bringup/support/owned_topics.cpp"
        )
    if not repo.exists("integrated_bringup/src/support/owned_topics.cpp"):
        failures.append("real path integrated_bringup/src/support/owned_topics.cpp not found")

    if failures:
        print("SELF-TEST FAILURES:", file=sys.stderr)
        for f in failures:
            print(f"  - {f}", file=sys.stderr)
        return 1
    total = len(GREP_CASES) + len(SLUG_CASES) + len(BRE_EXEC_CASES) + len(PARKED_PATTERN_CASES) + 4
    print(f"self-test: {total} fixtures passed")
    return 0


def repo_root() -> Path:
    out = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()
    return Path(out)


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter
    )
    ap.add_argument("--files", nargs="*", help="restrict the scan to these files (hook scope)")
    ap.add_argument("--self-test", action="store_true", help="run the checker's own fixtures")
    args = ap.parse_args(argv)

    if args.self_test:
        return self_test()

    repo = Repo(repo_root())
    if args.files:
        targets = []
        for f in args.files:
            p = Path(f)
            rel = p.resolve().relative_to(repo.root).as_posix() if p.is_absolute() else f
            if rel.endswith((".md", ".yml", ".yaml")) and (repo.root / rel).exists():
                targets.append(rel)
    else:
        targets = repo.corpus()

    findings: list[Finding] = []
    for rel in targets:
        findings.extend(check_file(repo, rel))

    findings.sort(key=lambda f: (f.path, f.line, f.code))
    for f in findings:
        print(str(f))
    if findings:
        by_code: dict[str, int] = {}
        for f in findings:
            by_code[f.code] = by_code.get(f.code, 0) + 1
        summary = ", ".join(f"{k}={v}" for k, v in sorted(by_code.items()))
        print(f"\n{len(findings)} finding(s) across {len(targets)} file(s) [{summary}]")
        return 1
    print(f"docs validation clean ({len(targets)} file(s))")
    return 0


if __name__ == "__main__":
    sys.exit(main())
