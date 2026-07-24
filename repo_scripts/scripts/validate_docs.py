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
D4  no ``file.cpp:123`` / ``#L123`` line-anchor citations into *code* in the
    agent corpus.  Line numbers drift on every edit; cite a symbol instead.
    Doc-to-doc references and URLs carrying a port are not that failure.
D7  ``detect`` fenced blocks: the pattern is linted for the escaping mistakes
    that make a grep silently match nothing, then run against a required
    ``# probe:`` line it must match (and any ``# antiprobe:`` it must not), so
    a pattern that compiles but can no longer fire still fails.  An optional
    ``# exemplar:`` additionally asserts the state of the tree today.
D8  no detection pattern parked in a markdown table cell.  A cell cannot hold
    an unescaped ``|``, so a regex put in one gets escaped into something
    inert; the table is the root cause, not the individual typos.
D9  no hardcoded package count in the constitution / README that disagrees with
    the tree (AP-DOC-1).  ``21개 ROS 2 패키지`` in ``README.md`` is accurate
    today but rots silently when a package is added or removed; the check
    compares the claimed number against the tracked ``package.xml`` count and
    fails on drift.  Scoped to root ``README.md`` / ``CLAUDE.md`` (per-package
    READMEs legitimately describe their own contents) and honours the inline
    ``allow D9`` marker for a genuine sub-total.

Both a static lint *and* a probe are required, and neither subsumes the other.
A doubled backslash (``\\\\b``) makes grep exit 2 -- loud.  But ``\\|`` inside
an ERE is *valid syntax* for a literal pipe: exit 1, no output,
indistinguishable from a clean tree.  Running the pattern alone would have
graded this corpus healthy.

Note what the probe is and is not.  The pattern is translated into Python's
regex dialect and run by :mod:`re`; grep is never invoked.  That catches a
pattern which has stopped matching what it is meant to match.  It cannot
observe grep-specific behaviour -- GNU grep tolerating an empty alternation
that the sandbox's ugrep rejects outright -- which is precisely why the static
lint carries that class of defect and is not optional.

Scope boundaries are deliberate, and narrower than "any grep anywhere".  D8
covers table cells plus already-mangled escaping; a grep quoted in a sentence
or shown inside a ``bash`` fence is documentation, not a rotting invariant, and
firing on those makes the gate a nuisance that gets switched off.  D3, D4 and
D8 all honour an inline ``<!-- validate-docs: allow D4 -->`` marker, covering
the line it sits on and the next, so a doc can discuss the anti-pattern it
documents without the only remedy being to delete the sentence.

Self-verification
-----------------
``--self-test`` runs two layers.  Helper fixtures encode the silent-pass inputs
a previous attempt at this validator got wrong (option clusters such as
``grep -rn --include='*.cpp' -E 'a\\|b'`` misgraded as BRE, the pattern
argument shadowed by ``--include``), plus GitHub slug conformance cases.
Document fixtures then drive :func:`check_markdown` over whole markdown inputs
and assert the exact finding codes -- both that each check fires on a known-bad
document and that it stays quiet on the near-miss beside it.

That second layer is not decoration.  With only the helper fixtures, severing
:func:`check_detect_blocks` or disabling the D4 loop left ``--self-test`` and
the corpus scan both reporting success: the checker could lose whole checks
silently, which is the same class of defect it exists to catch.  A checker for
silent failures cannot be trusted because it is green on the repository; it has
to be green on inputs whose answer is known, and red when it is broken.

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
    "--binary-files",
    "--devices",
    "--directories",
    "--label",
}
# NOT here: `--color` / `--colour` take an *optional* argument in GNU grep, which
# it only accepts when attached with `=`. Listing them as value-consuming made
# `grep --color -E '<pat>'` swallow the following `-E` as the colour value, so the
# flavor was misread as BRE and a dead ERE pattern (a literal `\|`) graded healthy
# -- a silent false pass in the checker whose whole job is catching those.

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

# D3 exemption: GitHub Actions runners really do live at /home/runner, so a doc
# quoting a CI log path is describing someone else's machine, not leaking its
# own.  This is a fixed, well-known location rather than a user's home.
CI_HOME_PREFIXES = ("/home/runner/",)

# D4: only *code* citations must be symbol-based.  The rule exists because line
# numbers drift on every edit and a citation into a source file silently starts
# pointing at the wrong statement; a doc-to-doc reference such as
# `handoff.md:1-9` and a quoted `path.yaml:3` in captured tool output are not
# that failure.  The leading lookbehind keeps a URL with a port
# (`http://host/a.py:8080`) out: those matched as line anchors before, and no
# rewording could satisfy the gate.
LINE_ANCHOR_RE = re.compile(r"(?<![\w:/-])[\w./-]+\.(?:cpp|hpp|h|cc|py|sh):\d+")
L_ANCHOR_RE = re.compile(r"#L\d+")
INLINE_GREP_RE = re.compile(r"(?:^|\|\s*)r?g?grep\s+-{1,2}\w")
BARE_PATH_RE = re.compile(r"(?<![\w./-])((?:[\w.-]+/)+[\w.-]+\.[A-Za-z0-9]+)")

# An explicit, greppable opt-out for the three checks that scan prose and so
# can collide with a doc that discusses the very thing they forbid -- quoting a
# diagnostic, showing the anti-pattern by example.  A gate with no override
# gets satisfied by deleting the sentence, which is worse than the finding.
# The marker covers the line it sits on and the line after it, so it can lead a
# block without interrupting the prose.
SUPPRESS_RE = re.compile(r"validate-docs:\s*allow\s+(D\d+(?:\s*,\s*D\d+)*)")

# Corpus whose code citations must be symbol-based (D4).
SYMBOL_CITATION_DIRS = ("agent_docs/", ".claude/")

# D9 scope: the constitution and the root README are the two documents whose
# stated package count is a whole-repo total that AP-DOC-1 warns will drift.
# A per-package README saying "이 패키지는 2개의 ..." is describing itself, not
# the repo, so scoping to these two avoids firing on legitimate sub-counts.
COUNT_SCOPED_DOCS = ("README.md", "CLAUDE.md")

# D9: "N개 [ROS 2] 패키지" -- a claim about how many packages the repo has.
# The counted noun must be 패키지; this deliberately does not match the prose
# "7개 목록" / "1개 이상" that appears in the constitution for unrelated reasons.
# `(\d+)` is greedy, so "210개" captures 210 (never a partial "21").
PACKAGE_COUNT_RE = re.compile(r"(\d+)\s*개\s*(?:의\s*)?(?:ROS[\s-]*2\s*)?패키지")


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


SETEXT_UNDERLINE_RE = re.compile(r"^(=+|-+)\s*$")


def _is_setext_heading(lines: list[tuple[str, bool]], idx: int) -> bool:
    """Is line ``idx`` a setext heading (text underlined by ``===`` / ``---``)?

    Deliberately conservative. The text must be a single non-blank line whose
    predecessor is blank or the file start, and the underline a homogeneous run
    of one character. That rejects the only ``-``/``=`` runs this corpus actually
    contains -- YAML front-matter delimiters (whose text line follows another
    non-blank key line) and table separators / thematic breaks -- rather than
    inventing anchors for them, which is the same false-positive class this
    validator exists to avoid.
    """
    raw, in_fence = lines[idx]
    if in_fence or not raw.strip() or idx + 1 >= len(lines):
        return False
    if HEADING_RE.match(raw) or FENCE_RE.match(raw):
        return False
    nxt, nxt_fence = lines[idx + 1]
    if nxt_fence or not SETEXT_UNDERLINE_RE.match(nxt.strip()):
        return False
    return idx == 0 or not lines[idx - 1][0].strip()


def anchors_of(text: str) -> set[str]:
    """Every anchor a rendered markdown document exposes."""
    anchors: set[str] = set()
    counts: dict[str, int] = {}

    def add(base: str) -> None:
        if not base:
            return
        n = counts.get(base, 0)
        counts[base] = n + 1
        anchors.add(base if n == 0 else f"{base}-{n}")

    lines = list(iter_lines_with_fence_state(text))
    for idx, (raw, in_fence) in enumerate(lines):
        if in_fence:
            continue
        m = HEADING_RE.match(raw)
        if m:
            add(slugify(m.group(2)))
        elif _is_setext_heading(lines, idx):
            add(slugify(raw))
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
    # ... and blank out bracket expressions too: inside `[...]` a '|' or ')' is
    # an ordinary member of the set, so `x[|)]y` is not an empty group.
    bare = re.sub(r"\\.", "\x00", pattern)
    bare = re.sub(r"\[\^?\]?[^]]*\]", lambda m: "\x00" * len(m.group(0)), bare)
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


# POSIX character classes are ordinary in a grep pattern and meaningless to
# Python's `re`, which reads `[[:space:]]` as "the set { [ : s p a c e }" and
# warns about a nested set.  Left untranslated, a perfectly good detect pattern
# was reported as no longer matching its own probe -- the validator accusing the
# author of the exact rot it exists to find.
POSIX_CLASSES = {
    "alpha": "a-zA-Z",
    "digit": "0-9",
    "alnum": "a-zA-Z0-9",
    "space": r" \t\n\r\f\v",
    "blank": r" \t",
    "upper": "A-Z",
    "lower": "a-z",
    "punct": r"!-/:-@\[-`{-~",
    "word": r"\w",
    "xdigit": "0-9A-Fa-f",
}
POSIX_CLASS_RE = re.compile(r"\[:(" + "|".join(POSIX_CLASSES) + r"):\]")


def _translate_bre(src: str) -> str:
    """Rewrite a BRE into Python's (ERE-like) dialect, one token at a time.

    In BRE the grouping/alternation metacharacters are the *escaped* forms
    (``\\(`` ``\\)`` ``\\{`` ``\\}`` ``\\|``); the bare forms are literals -- the
    exact inversion of Python's engine.  A char-by-char pass is the only correct
    way to do this: the earlier blanket ``str.replace`` mis-handled an escaped
    backslash directly before a paren/pipe (``\\\\(``), eating the second
    backslash of the pair and silently dropping the literal backslash the
    pattern was meant to match.
    """
    out: list[str] = []
    i, n = 0, len(src)
    while i < n:
        c = src[i]
        if c == "\\" and i + 1 < n:
            nxt = src[i + 1]
            if nxt in "(){}|":
                out.append(nxt)  # BRE metachar -> Python metachar
            elif nxt == "\\":
                out.append("\\\\")  # literal backslash, kept intact
            else:
                out.append("\\" + nxt)  # other escape passes through (\. \b ...)
            i += 2
            continue
        if c in "(){}|":
            out.append("\\" + c)  # bare form is a literal in BRE
        else:
            out.append(c)
        i += 1
    return "".join(out)


def compile_pattern(flavor: str, pattern: str) -> tuple[re.Pattern | None, str | None]:
    """Translate an ERE/BRE pattern into Python's regex dialect and compile it.

    Note what this is and is not: the probe check runs the pattern through
    Python's engine, not through grep.  It catches a pattern that has stopped
    matching what it is supposed to match.  It cannot observe grep-specific
    behaviour -- GNU grep tolerating an empty alternation where the sandbox's
    ugrep rejects it, say -- which is why :func:`lint_pattern` is a separate,
    non-negotiable static pass and not a fallback for when execution is
    inconvenient.
    """
    if flavor == "fixed":
        return re.compile(re.escape(pattern)), None
    src = POSIX_CLASS_RE.sub(lambda m: POSIX_CLASSES[m.group(1)], pattern)
    if flavor == "bre":
        src = _translate_bre(src)
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

    def package_count(self) -> int:
        """Tracked ROS 2 packages == tracked ``package.xml`` files (D9 ground truth)."""
        return sum(1 for f in self.files if f == "package.xml" or f.endswith("/package.xml"))


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


def suppressions(text: str) -> dict[int, set[str]]:
    """Map line number -> codes suppressed there by an inline marker."""
    out: dict[int, set[str]] = {}
    for lineno, raw in enumerate(text.splitlines(), 1):
        m = SUPPRESS_RE.search(raw)
        if not m:
            continue
        codes = {c.strip() for c in m.group(1).split(",")}
        out.setdefault(lineno, set()).update(codes)
        out.setdefault(lineno + 1, set()).update(codes)
    return out


def check_markdown(repo: Repo, rel: str, text: str) -> list[Finding]:
    findings: list[Finding] = []
    base = Path(rel).parent
    in_symbol_corpus = rel.startswith(SYMBOL_CITATION_DIRS)
    allowed = suppressions(text)

    for lineno, (raw, in_fence) in enumerate(iter_lines_with_fence_state(text), 1):
        ok = allowed.get(lineno, frozenset())
        # D3 absolute private paths: checked inside fences too -- a shell
        # snippet is the copy-paste surface that actually breaks for others.
        if "D3" not in ok:
            for m in HOME_PATH_RE.finditer(raw):
                if m.group(0).startswith(CI_HOME_PREFIXES):
                    continue
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

        # A table row is the root cause D8 exists for: the cell cannot hold an
        # unescaped '|', so a pattern parked in one gets mangled into something
        # inert.  Prose is not that hazard.
        in_table = raw.lstrip().startswith("|")
        for m in CODE_SPAN_RE.finditer(raw):
            findings.extend(check_path_token(repo, rel, lineno, m.group(1), "D1"))
            if (
                in_symbol_corpus
                and "D8" not in ok
                and is_parked_detection_pattern(m.group(1), in_table=in_table)
            ):
                findings.append(
                    Finding(
                        rel,
                        lineno,
                        "D8",
                        "detection pattern parked outside a ```detect block -- a "
                        "markdown table cell cannot hold an unescaped '|', so "
                        "a pattern in one gets escaped into something "
                        "that silently matches nothing.  Move it to a fenced "
                        "detect block, where it is linted and probed.",
                    )
                )

        if in_symbol_corpus and "D4" not in ok:
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
    if rel in COUNT_SCOPED_DOCS:
        findings.extend(check_package_count(rel, text, repo.package_count(), allowed))
    return findings


def check_package_count(
    rel: str, text: str, actual: int, allowed: dict[int, set[str]]
) -> list[Finding]:
    """D9: a hardcoded "N개 패키지" claim must equal the tracked package.xml count.

    ``actual`` is passed in rather than measured here so the check is hermetic
    under ``--self-test`` -- a fixture states a known count and a known claim,
    and the answer does not move when the real tree gains a package.
    """
    if rel not in COUNT_SCOPED_DOCS:
        return []
    findings: list[Finding] = []
    for lineno, raw in enumerate(text.splitlines(), 1):
        if "D9" in allowed.get(lineno, frozenset()):
            continue
        for m in PACKAGE_COUNT_RE.finditer(raw):
            if int(m.group(1)) != actual:
                findings.append(
                    Finding(
                        rel,
                        lineno,
                        "D9",
                        f"hardcoded package count '{m.group(0)}' != {actual} tracked "
                        "package.xml files -- AP-DOC-1: cite how to measure "
                        "(find -name package.xml) or fix the number",
                    )
                )
    return findings


def mask_code_spans(raw: str) -> str:
    """Blank out inline code spans, preserving offsets."""
    return CODE_SPAN_RE.sub(lambda m: " " * len(m.group(0)), raw)


def is_parked_detection_pattern(span: str, in_table: bool) -> bool:
    """Is this inline code span an invariant *detection* grep at risk of rot?

    Two conditions, and both matter.  The pattern has to actually carry
    alternation -- that is the construct a markdown table cell cannot hold, and
    the reason a parked pattern degrades into `\\|`, a literal pipe that matches
    nothing.  And it has to be *in* a table cell, or already be carrying the
    damage.

    The earlier form asked only "is this an ERE or PCRE grep", which the
    docstring justified by saying those are the ones that carry alternation --
    but it never checked, so every `grep -P '\\tTAB'` quoted in prose was
    reported as a rotting invariant detector, complete with a message about
    table cells that had nothing to do with the line.  A gate that fires on a
    debugging tip in a sentence is one that gets switched off.

    Prose greps in a plain paragraph are therefore out of scope, deliberately.
    So are patterns inside a non-``detect`` fenced block: docs legitimately show
    shell sessions, and reading those as parked invariants would re-create the
    same nuisance one layer down.  D8 covers table cells and already-mangled
    escaping; it does not claim more.
    """
    span = span.strip()
    if not INLINE_GREP_RE.match(span):
        return False
    flavor, pattern, error = parse_grep(span)
    if error is not None or pattern is None or flavor not in ("ere", "pcre"):
        return False
    # Already mangled: report wherever it sits, table or not.
    if r"\|" in pattern or re.search(r"\\\\[A-Za-z(){}.[\]]", pattern):
        return True
    return in_table and "|" in pattern


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
    # A *link* must resolve to a real path on disk.  The suffix index exists so
    # that a bare include-shorthand token in a code span
    # (`rtc_base/types/types.hpp`) resolves to the header it names; letting a
    # link ride it meant a nonexistent target could pass D1 and then be reported
    # by D2 as an anchor missing from a file that is not there at all.
    if not (repo.root / normalised).exists():
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
            # A workflow legitimately names the runner's home (/home/runner/work,
            # caches, tool paths); that is someone else's fixed CI machine, not a
            # leak of this user's home. Same carve-out check_markdown applies.
            if m.group(0).startswith(CI_HOME_PREFIXES):
                continue
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

    # `expect` pairs with `exemplar` by position, falling back to the first (or
    # "match") when fewer are given. Reading only expects[0] for every exemplar
    # made "matches in A, none in B" impossible to express, and silently
    # mis-graded one of the two.
    expects = directives.get("expect") or []
    for idx, exemplar in enumerate(directives.get("exemplar", [])):
        expect = expects[idx] if idx < len(expects) else (expects[0] if expects else "match")
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

# Setext headings expose anchors too; the `---`/`===` runs that are NOT headings
# (front-matter delimiters, table separators, thematic breaks) must stay silent.
SETEXT_CASES: list[tuple[str, set[str]]] = [
    ("Overview\n========\n", {"overview"}),
    ("Sub Heading\n-----------\n", {"sub-heading"}),
    ("---\ntitle: X\n---\n\n# Real\n", {"real"}),  # YAML front-matter is not a heading
    ("| a | b |\n| --- | --- |\n", set()),  # table separator row
    ("Some text.\n\n---\n", set()),  # thematic break after a blank line
]

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
    # An escaped backslash right before a paren/pipe is a LITERAL backslash
    # followed by a literal paren/pipe, not a group. The old placeholder-swap
    # translation dropped the backslash and matched a bare paren.
    ("bre", r"\\(", r"\(", True),  # matches backslash-then-paren
    ("bre", r"\\(", "(", False),  # must NOT match a bare paren
    ("bre", r"\\|", r"\|", True),  # backslash-then-pipe, literal
    ("bre", r"\\|", "x", False),  # not alternation, so "x" alone never matches
]


# (span, in_table, parked?)
PARKED_PATTERN_CASES: list[tuple[str, bool, bool]] = [
    ("grep -nE 'RCLCPP_(INFO|WARN)\\(' <RT file>", True, True),
    ("grep -rniE '(ur5e|iiwa7)' rtc_*/", True, True),
    ("grep -rl ENABLE_COVERAGE */CMakeLists.txt", True, False),
    ("grep -n 'struct.*Data' <header>", True, False),
    ("colcon test --packages-select rtc_base", True, False),
    ("ament_add_gtest(... ENV ROS_DOMAIN_ID=<n>)", True, False),
    # The same alternating pattern in a *sentence* is a debugging tip, not an
    # invariant parked in a cell that will mangle it.
    ("grep -rniE '(ur5e|iiwa7)' rtc_*/", False, False),
    # No alternation at all: nothing a table cell can damage.
    ("grep -P '\\tTAB' src/", True, False),
    ("grep -nE 'RCLCPP_INFO' <RT file>", True, False),
    # Already carrying the damage -- report it wherever it sits.
    ("grep -rnE 'a\\|b' src/", False, True),
]


# Whole-document fixtures: (name, rel, markdown, sorted codes expected).
#
# Everything above tests a pure helper.  That left the part of this validator
# that actually decides things -- check_markdown, and the checks it dispatches
# to -- with no fixture at all, so a check could be deleted outright and both
# `--self-test` and the corpus scan would still report success.  That was
# verified, not theorised: severing check_detect_blocks and disabling the D4
# loop left every count green.  A checker for silent failures that is itself
# able to fail silently proves nothing, so each check now has to demonstrate it
# fires on a document whose answer is known, and stays quiet on the
# near-miss beside it.
DOC_FIXTURES: list[tuple[str, str, str, list[str]]] = [
    ("clean", "agent_docs/f.md", "# Title\n\nSee `RtControllerNode::Tick`.\n", []),
    ("D1 broken link", "agent_docs/f.md", "[gone](./nope.md)\n", ["D1"]),
    ("D1 live link", "agent_docs/f.md", "[arch](architecture.md)\n", []),
    (
        "D2 bogus anchor",
        "agent_docs/f.md",
        "[arch](architecture.md#no-such-heading-here)\n",
        ["D2"],
    ),
    ("D3 home path", "agent_docs/f.md", "cd /home/someone/ws\n", ["D3"]),
    ("D3 CI runner path is not a leak", "agent_docs/f.md", "at /home/runner/work/x\n", []),
    (
        "D3 suppressed",
        "agent_docs/f.md",
        "<!-- validate-docs: allow D3 -->\nNever write /home/junho/ws in a launch file.\n",
        [],
    ),
    ("D4 code line anchor", "agent_docs/f.md", "see rt_controller_node.cpp:120\n", ["D4"]),
    ("D4 #L anchor", "agent_docs/f.md", "cited as architecture.md#L12 upstream\n", ["D4"]),
    ("D4 doc range is not a code citation", "agent_docs/f.md", "see handoff.md:1-9\n", []),
    ("D4 URL port is not a line anchor", "agent_docs/f.md", "at http://host/a.py:8080\n", []),
    (
        "D4 suppressed",
        "agent_docs/f.md",
        "<!-- validate-docs: allow D4 -->\nThe hook printed `verify-changes.sh:42: error`.\n",
        [],
    ),
    ("D4 is scoped to the agent corpus", "docs/f.md", "see rt_controller_node.cpp:120\n", []),
    (
        "D8 pattern parked in a table cell",
        "agent_docs/f.md",
        "| RT-1 | `grep -rnE 'RCLCPP_(INFO|WARN)' src/` |\n",
        ["D8"],
    ),
    (
        "D8 ignores the same grep in prose",
        "agent_docs/f.md",
        "Run `grep -rnE 'RCLCPP_(INFO|WARN)' src/` to find them.\n",
        [],
    ),
    (
        "D8 reports already-mangled escaping anywhere",
        "agent_docs/f.md",
        "Run `grep -rnE 'a\\|b' src/` to find them.\n",
        ["D8"],
    ),
    (
        "D7 pattern that no longer matches its probe",
        "agent_docs/f.md",
        "```detect id=RT-X\ngrep -rnE 'RCLCPP_' src/\n# probe:   printf(\"hi\");\n```\n",
        ["D7"],
    ),
    (
        "D7 missing probe",
        "agent_docs/f.md",
        "```detect id=RT-X\ngrep -rnE 'RCLCPP_' src/\n```\n",
        ["D7"],
    ),
    (
        "D7 escaped pipe in an ERE is caught statically",
        "agent_docs/f.md",
        "```detect id=RT-X\ngrep -rnE 'a\\|b' src/\n# probe:   a|b\n```\n",
        ["D7"],
    ),
    (
        "D7 POSIX class is a valid grep pattern",
        "agent_docs/f.md",
        "```detect id=RT-X\ngrep -rnE '^[[:space:]]*RCLCPP_' src/\n"
        "# probe:      RCLCPP_INFO(x);\n# antiprobe: int x = 1;\n```\n",
        [],
    ),
    (
        "D7 antiprobe catches an over-broad pattern",
        "agent_docs/f.md",
        "```detect id=RT-X\ngrep -rnE 'RCLCPP_' src/\n# probe: RCLCPP_INFO(x);\n"
        "# antiprobe: RCLCPP_INFO(y);\n```\n",
        ["D7"],
    ),
    (
        "code span holding C++ is not a link",
        "agent_docs/f.md",
        "| `operator[](name)` | insertion order |\n",
        [],
    ),
    # D9 end-to-end severance guard: 999 is never the real package count, so
    # this stays red as long as check_package_count is wired into
    # check_markdown -- severing it turns the fixture (and the suite) green.
    (
        "D9 package count drift",
        "README.md",
        "본 저장소는 999개 ROS 2 패키지로 구성됩니다.\n",
        ["D9"],
    ),
    # ...and it must not fire on a count of a *different* noun, count-independent.
    (
        "D9 ignores a non-package count",
        "README.md",
        "빌드 후 999개 파일이 생성됩니다.\n",
        [],
    ),
]


# Hermetic count fixtures: (rel, markdown, actual_count, sorted codes expected).
# The count is stated per-fixture so the known answer does not drift with the
# real tree -- the layer that turns "silent silent-pass" red for D9.
COUNT_CASES: list[tuple[str, str, int, list[str]]] = [
    ("README.md", "21개 ROS 2 패키지로 구성됩니다.\n", 21, []),  # match -> silent
    ("README.md", "20개 ROS 2 패키지로 구성됩니다.\n", 21, ["D9"]),  # drift -> red
    ("README.md", "21개 패키지\n", 21, []),  # bare form, match
    ("README.md", "22개 패키지\n", 21, ["D9"]),  # bare form, drift
    ("README.md", "210개 ROS 2 패키지\n", 21, ["D9"]),  # greedy: 210, not a partial 21
    ("README.md", "빌드 후 21개 파일\n", 21, []),  # 파일, not 패키지 -> silent
    ("CLAUDE.md", "현재 45개 패키지 상수가 있다\n", 21, ["D9"]),  # re-introduced in constitution
    ("agent_docs/x.md", "20개 ROS 2 패키지\n", 21, []),  # out of D9 scope -> silent
    (
        "README.md",
        "<!-- validate-docs: allow D9 -->\n형상 추정 2개 패키지\n",
        21,
        [],
    ),  # legitimate sub-count, suppressed
]


def self_test() -> int:
    failures: list[str] = []

    repo_for_docs = Repo(repo_root())
    for name, rel, body, want_codes in DOC_FIXTURES:
        got_codes = sorted(f.code for f in check_markdown(repo_for_docs, rel, body))
        if got_codes != sorted(want_codes):
            failures.append(f"doc fixture {name!r}: codes={got_codes}, want {sorted(want_codes)}")

    for rel, body, actual, want in COUNT_CASES:
        got_codes = sorted(
            f.code for f in check_package_count(rel, body, actual, suppressions(body))
        )
        if got_codes != sorted(want):
            failures.append(
                f"count case {rel} {body!r} actual={actual}: codes={got_codes}, want {sorted(want)}"
            )

    for span, in_table, want in PARKED_PATTERN_CASES:
        got = is_parked_detection_pattern(span, in_table=in_table)
        if got != want:
            failures.append(
                f"is_parked_detection_pattern({span!r}, in_table={in_table}) = {got}, want {want}"
            )

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

    for stext, swant in SETEXT_CASES:
        sgot = anchors_of(stext)
        if sgot != swant:
            failures.append(f"anchors_of({stext!r}) = {sorted(sgot)}, want {sorted(swant)}")

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
    total = (
        len(DOC_FIXTURES)
        + len(COUNT_CASES)
        + len(GREP_CASES)
        + len(SLUG_CASES)
        + len(BRE_EXEC_CASES)
        + len(PARKED_PATTERN_CASES)
        + len(SETEXT_CASES)
        + 4
    )
    print(f"self-test: {total} fixtures passed")
    return 0


def repo_root() -> Path:
    out = subprocess.run(
        ["git", "rev-parse", "--show-toplevel"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout.strip()
    # resolve() so that `Path(f).resolve().relative_to(repo.root)` in main() holds
    # when a component of the workspace path is a symlink: git may report the
    # symlink path while Path.resolve() canonicalises it, and the mismatch would
    # reject a file that is genuinely inside the repo (ValueError -> exit 2).
    return Path(out).resolve()


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
    if args.files is not None:
        # `is not None`, not truthiness: `--files` with no operands means "no
        # files to check", and falling through to the full corpus scan there
        # would let a defect in an untouched file block a turn that never went
        # near it -- the one thing the hook scope exists to prevent.
        targets = []
        for f in args.files:
            # Relative entries resolve against the caller's cwd, not blindly
            # against the repo root: interpreting them as root-relative meant an
            # invocation from a subdirectory silently checked nothing and
            # reported success.
            try:
                rel = Path(f).resolve().relative_to(repo.root).as_posix()
            except ValueError:
                print(f"--files: '{f}' is outside the repository", file=sys.stderr)
                return 2
            if not rel.endswith((".md", ".yml", ".yaml")):
                continue
            # A path that is gone was deleted in this change set; that is not an
            # error, and the hook passes deleted docs through routinely.
            if (repo.root / rel).exists():
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
