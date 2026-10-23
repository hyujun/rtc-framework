#!/usr/bin/env python3
"""Documentation-corpus validator (issue #213).

The agent-facing documentation in this repository carries executable claims:
relative links, code-location citations, and the invariant *detection patterns*
that the constitution tells an agent to run as a pre-commit self-check.  None of that
is covered by ``colcon test``, and the CI workflow explicitly ``paths-ignore``s
``agent_docs/**``.  When a detection pattern rots, it does not fail loudly --
``grep`` exits 1 with no output, which reads exactly like "no violations".
This validator turns that silence into a red build.

Checks
------
D1  link / file-reference target exists (markdown links + path-shaped inline
    code spans).  There is no allow marker: a history line that names a file
    since deleted cites it by name or as ``git show <commit>:<path>`` (the
    form docs/dynamic_catching/README.md uses), which stays true.  Header include shorthand (``rtc_base/types/types.hpp`` for
    ``rtc_base/include/rtc_base/types/types.hpp``) resolves via path-component
    suffix matching, so a genuinely wrong path such as
    ``integrated_bringup/support/owned_topics.cpp`` (missing ``src/``) is still
    reported.
D2  in-repo anchor exists, using GitHub's heading-slug algorithm (see
    :func:`slugify` -- ``## Build & Test`` really is ``#build--test``).
D3  no link escapes the repository, and no ``/home/<user>/`` absolute path
    appears anywhere in the corpus (including shell snippets, which get copied).
D4  no ``file.cpp:123`` / ``#L123`` line-anchor citations into *code* in the
    agent corpus -- ``agent_docs/``, ``.claude/`` and the documents the
    constitution's detail lives in (``CONSTITUTION_COMPANION_DOCS``).  Line
    numbers drift on every edit; cite a symbol instead.
    Doc-to-doc references and URLs carrying a port are not that failure.
D7  ``detect`` fenced blocks: the pattern is linted for the escaping mistakes
    that make a grep silently match nothing, then run against a required
    ``# probe:`` line it must match (and any ``# antiprobe:`` it must not), so
    a pattern that compiles but can no longer fire still fails.  An optional
    ``# exemplar:`` additionally asserts the state of the tree today.
D10 no bare "§N.M" section ref in the constitution corpus or its companions
    (same set as D4).  The same number
    means different things in AGENTS.md, in the compliance normative spec, and
    in a file's own numbered headings; a prefix is what tells them apart.
D11 no rule-ID reference to an ID its owning file never defines (RT-/ARCH-/
    PROC-/NUM-/E- own by invariants.md, AP- by docs/reference/anti-patterns.md,
    P1..P5 by design-principles.md).
D12 the two constitutions (``CLAUDE.md`` / ``AGENTS.md``) stay inside a size
    budget: at most 200 lines, at most 18 KiB, and no prose line over 500
    characters (table rows and fenced blocks are exempt).  Anthropic's guidance
    is "under 200 lines"; the line cap alone was being satisfied by making the
    lines longer (same line count, bytes up 70% in two months), so bytes and
    line length are gated as well.  Honours ``allow D12`` for a single line.
    The same code carries a per-file byte budget for the whole constitutional
    corpus -- the two constitutions, ``agent_docs/*.md`` and
    ``.claude/rules/*.md`` -- and for every ``.claude/skills/**.md``, read from
    ``repo_scripts/config/docs_budget.yaml``.
    The caps above never covered ``agent_docs/``, which more than doubled in
    ten weeks while the constitutions sat pinned at theirs: the content the cap
    pushed out of the constitution landed one directory down, ungated.  A file
    over its budget, a corpus file with no entry, and an entry whose file is
    gone all fail.  The numbers live in a data file rather than here so that
    lowering one as a document shrinks is a config edit, not a validator edit
    (the Stop hook still keys repo_scripts on its whole tree, so the edit owes
    that package's verdict like any other).
    Given ``--budget-base REV``, a budget higher than it is at REV also fails
    unless its line says ``# raised from <old>: <why>`` -- one tree cannot show
    that a number was raised to make a finding go away; the base can.
    ``agent_docs/*.md`` additionally carries the constitutions' 500-character
    prose-line cap and a 500-character cap on a single table CELL (a row may be
    longer than that; a cell may not).  A byte budget says how big a document
    is, not what shape it has: one cell of controllers.md had grown past 4,000
    characters, holding a dozen rules nobody could find or cite, while its row
    count stayed flat.  Fenced blocks are exempt; ``allow D12`` covers a line.
D13 every ``CLAUDE.md §N[.M]`` / ``AGENTS.md §N[.M]`` reference -- plain or in
    link form, in the docs and in the comments of every tracked source / config
    file (see :func:`is_section_ref_source`) -- names a numbered heading the
    target file actually has, and a bare ``§N`` inside a constitution resolves
    against that file's own headings.  D10 only checks that a ref carries a
    namespace; a renumbering keeps the namespace and loses the section, which is
    exactly the drift D10 cannot see.  It checks that the section EXISTS, not
    that it is the right one: a ref re-pointed onto another real section passes.
    Merging CLAUDE.md into AGENTS.md left six such refs green (a quoted
    subsection title under the wrong number, a pointer to prose that had moved
    into a hook header, an owner claim for content the section had handed to
    each tool's own docs), so review re-pointed refs against their context by
    hand.  ``--section-refs`` runs D13 alone -- the Stop hook calls it over the
    whole corpus when a constitution's numbered headings change, because the
    refs a renumbering breaks live in files the change never touched.

D8  no detection pattern parked in a markdown table cell.  A cell cannot hold
    an unescaped ``|``, so a regex put in one gets escaped into something
    inert; the table is the root cause, not the individual typos.
D9  no hardcoded package count in the constitution / README that disagrees with
    the tree (AP-DOC-1).  A phrase like ``21개 ROS 2 패키지`` is accurate the
    day it is written but rots silently when a package is added or removed; the check
    compares the claimed number against the tracked ``package.xml`` count and
    fails on drift.  Scoped to root ``README.md`` / ``CLAUDE.md`` (per-package
    READMEs legitimately describe their own contents) and honours the inline
    ``allow D9`` marker for a genuine sub-total.
D14 every git-tracked ``README.md`` has a byte budget in
    ``repo_scripts/config/readme_budget.yaml``, and stays inside it.  READMEs had
    become the default destination for every doc update: 1.46 MB over 27 files,
    one 148 KB README carrying a 94 KB annotated file tree, one table cell holding
    a paragraph.  D12 budgets only the constitutional corpus and exempts fenced
    blocks, so none of that was gated.  The file has the same ``path: bytes`` shape
    and the same ratchet as ``docs_budget.yaml`` -- a README over budget, a tracked
    README with no entry and an entry whose README is gone fail, and under
    ``--budget-base REV`` a raise needs ``# raised from <old>: <why>`` on its line
    (one implementation, :func:`check_budget_raises`, for both files).
    An entry may end in the word ``strict``; only those READMEs get the shape
    checks, because 155 table cells over 200 characters and 140 prose lines over
    500 exist today and are rewritten phase by phase (#869) -- a README is marked
    strict in the change that brings it into shape.  Strict means: no table cell
    over 200 characters, no prose line over 500, every file-tree fence at most 60
    columns wide and 2 levels deep, exactly one H1 and it comes first, the H2s
    drawn from README_H2_ORDER without repeats and in that order, and no bare
    issue number (``#123``) in the body outside ``## 더 읽기`` and headings (a
    heading may keep one because anchors cite it).  Honours ``allow D14`` for a
    single line.
D15 a README section cited BY NAME resolves.  ``repo_scripts/README.md "빌드
    병렬도와 메모리"``, ``README §흔한 실패와 감별``, ``integrated_bringup README
    §Scene overlay``, ``[x](../rtc_tools/README.md) §포구 평가 도구`` and
    ``se3/README "Limits"`` name a section by its title; D2 only follows ``#anchor``
    links, so renaming or moving the heading breaks them without a sound.  The
    cited README's heading texts (H1-H6) and bold spans are the names a ref may
    use.  After ``§`` the extent of the name is unknown (prose continues), so it
    resolves when some name is a prefix of the rest of the line (cut at ``|`` in a
    table); a quoted ref resolves when it equals a name or extends one.  A ref
    that starts with a digit is a numbered ref (D10 / D13).  The cited README is
    found relative to the citing file, then the repo root, then by unique suffix
    among tracked READMEs; a bare ``README`` means the nearest ancestor README;
    what cannot be resolved is D1's business and is skipped.  Scanned: tracked
    markdown (outside fences) and the same non-markdown files as D13, whole text.
    Only README targets are in scope (README_REF_TARGET_RE, the one place to widen
    it -- issue #849).  Honours ``allow D15``.  ``--files`` checks the refs FROM
    the given files; a heading rename that breaks refs in files nobody touched is
    caught by the full scan in CI.

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
import functools
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

# Where the constitution's detail lives: the catalogues, procedures, rationale
# and the anti-pattern case book that agent_docs/ points at.  Not rule sources,
# so no byte budget -- but the constitution sends its reader here, the case book
# still owns the AP IDs (D11) and still carries detection greps, and all of it
# was inside the D4 / D8 / D10 scopes until it moved out of agent_docs/.  The
# move must not be what takes those checks off it.
CONSTITUTION_COMPANION_DOCS = (
    "docs/reference/",
    "docs/testing.md",
    "docs/controllers.md",
    "docs/modification-procedures.md",
)

# Corpus whose code citations must be symbol-based (D4), and where a detection
# pattern may not be parked in a table cell (D8).
SYMBOL_CITATION_DIRS = ("agent_docs/", ".claude/", *CONSTITUTION_COMPANION_DOCS)

# D9 scope: the two constitutions (CLAUDE.md for Claude Code, AGENTS.md for
# every other tool) and the root README are the documents whose stated package
# count is a whole-repo total that AP-DOC-1 warns will drift.  A per-package
# README saying "이 패키지는 2개의 ..." is describing itself, not the repo, so
# scoping to these three avoids firing on legitimate sub-counts.  AGENTS.md was
# outside this scope while it was a hand-maintained mirror of CLAUDE.md, which
# is exactly the file most likely to carry a stale copy of such a count.
COUNT_SCOPED_DOCS = ("README.md", "CLAUDE.md", "AGENTS.md")

# D9: "N개 [ROS 2] 패키지" -- a claim about how many packages the repo has.
# The counted noun must be 패키지; this deliberately does not match the prose
# "7개 목록" / "1개 이상" that appears in the constitution for unrelated reasons.
# `(\d+)` is greedy, so "210개" captures 210 (never a partial "21").
PACKAGE_COUNT_RE = re.compile(r"(\d+)\s*개\s*(?:의\s*)?(?:ROS[\s-]*2\s*)?패키지")


# D10: a bare "§N.M" with no namespace in front of it.  The corpus uses numeric
# section numbers for three unrelated things -- the constitutions' own sections
# (AGENTS.md §6), the compliance normative spec (§6.5 = the sigma_min-adaptive
# DLS lambda rule), and file-internal named sections (§RT Path Invariants).
# "§6.5" alone meant Sprint Contract in one file and a damping law in another,
# and both spellings coexisted inside invariants.md with nothing to tell them
# apart.  A reference is disambiguated when it is preceded by a markdown link
# (`...md) §3.5`), a document name (`AGENTS.md §6.5`), or the literal
# `compliance ` prefix for the normative spec.
# Scope: the constitution corpus and its companions.  A per-package doc citing
# "§3.9" next to the spec that owns §3.9 is unambiguous in context; the collision that D10
# exists for is the constitution corpus, where AGENTS.md's own section numbers
# and the compliance spec's both appear -- inside one file, with nothing to tell
# them apart.  Widening this to every README turned it into 159 findings that
# were almost all legitimate self-references.
SECTION_REF_SCOPED_DOCS = (
    "agent_docs/",
    "CLAUDE.md",
    "AGENTS.md",
    ".claude/",
    *CONSTITUTION_COMPANION_DOCS,
)
SECTION_REF_RE = re.compile(r"§\d+\.\d[\d.]*")
SELF_NUMBERED_HEADING_RE = re.compile(r"^#{2,3}\s+\d+\.\s", re.M)
SECTION_REF_QUALIFIED_RE = re.compile(
    r"(?:compliance|CLAUDE\.md|AGENTS\.md|README\.md|[A-Za-z0-9_.-]+\.md|\))`?\s*$"
)

# D11: a rule-ID reference to an ID that no document defines.  Each namespace
# has exactly one owner: invariants.md defines RT-/ARCH-/PROC-/NUM-/E-,
# docs/reference/anti-patterns.md defines AP- (the case book sits outside the
# constitution; the norms its cases produced are rows in invariants.md, keyed by
# the AP ID), design-principles.md defines P1..P5.  A typo'd
# or invented ID ("P6", "RT-11") reads as authoritative and sends the reader
# looking for a rule that was never written -- P1..P5 were referenced as IDs by
# two documents while the owning file had never assigned those labels at all.
RULE_ID_RE = re.compile(
    r"\b(RT-HOST-\d+|RT-\d+|ARCH-\d+|PROC-\d+|NUM-\d+[ab]?|E-\d+|AP-[A-Z]+-\d+|P[1-9])\b"
)
RULE_ID_OWNERS = {
    "RT": "agent_docs/invariants.md",
    "ARCH": "agent_docs/invariants.md",
    "PROC": "agent_docs/invariants.md",
    "NUM": "agent_docs/invariants.md",
    "E": "agent_docs/invariants.md",
    "AP": "docs/reference/anti-patterns.md",
    "P": "agent_docs/design-principles.md",
}
# IDs that are deliberately retired: referenced in prose as history, never as a
# live rule.  Keeping them listed is what lets D11 stay strict about the rest.
RULE_ID_RETIRED = frozenset({"RT-7", "AP-RTT-3", "AP-RTT-4", "AP-CTRL-2", "AP-CTRL-4"})

# D12: size budget for the two constitutions.  Both files are loaded into
# every session (CLAUDE.md directly, AGENTS.md through the `@AGENTS.md`
# import), so their size is the one documentation cost paid on every request.
# Each also has a smaller per-file budget below; these figures are the ceiling
# under it -- a budget is a number in a data file, and this is what bounds how
# far that number can be raised without editing the checker.
CONSTITUTION_DOCS = ("CLAUDE.md", "AGENTS.md")
CONSTITUTION_MAX_LINES = 200
CONSTITUTION_MAX_BYTES = 18 * 1024
CONSTITUTION_MAX_LINE_CHARS = 500
# D12, per-file budgets: every document an agent is told to treat as a rule
# source has a byte budget in SIZE_BUDGET_FILE, and so does every skill file:
# a skill is loaded whole when invoked and routes to the documents that own
# its steps, and unbudgeted, one grew from 3 KB into a 34 KB copy of them
# (issue comments and package docs) with nothing firing.  One "path: bytes"
# mapping per line -- valid YAML, parsed here by regex so the validator keeps
# running on a host without PyYAML (the CI job installs nothing for it).
SIZE_BUDGET_FILE = "repo_scripts/config/docs_budget.yaml"
SIZE_BUDGET_DIRS = ("agent_docs/", ".claude/rules/", ".claude/skills/")
SIZE_BUDGET_LINE_RE = re.compile(r"^([^\s#:][^:]*?)\s*:\s*(\d+)(?:\s+(strict))?\s*(?:#.*)?$")
# D12, the ratchet: raising the number is the cheapest way to make a budget
# finding go away, and a scan of one tree cannot tell a raised number from an
# original one.  Against a base revision (--budget-base) a raise passes only
# when its entry line says so, naming the value it was raised from -- a note
# left over from an earlier raise names a different value and authorises nothing.
BUDGET_RAISE_NOTE_RE = re.compile(r"#\s*raised from (\d+):\s*\S")
# D12, line shape: the rule documents share the constitutions' prose-line cap
# and add one for a table cell.  .claude/rules/ is left out on purpose -- a rule
# is a handful of judgement paragraphs written to be read whole on load, and its
# byte budget already bounds it.
LINE_CAP_DIRS = ("agent_docs/",)
MAX_CELL_CHARS = 500
TABLE_CELL_SPLIT_RE = re.compile(r"(?<!\\)\|")

# D14: the README corpus.  Same "path: bytes" file shape and the same raise
# ratchet as D12, one entry per tracked file whose basename is exactly README.md;
# an entry ending in `strict` also gets the shape checks below.  The caps are
# deliberately tighter than the rule documents' (README_MAX_CELL_CHARS vs
# MAX_CELL_CHARS): a README is read by a person scanning for one fact.
README_BUDGET_FILE = "repo_scripts/config/readme_budget.yaml"
README_NAME = "README.md"
README_MAX_CELL_CHARS = 200
README_MAX_PROSE_CHARS = 500
README_TREE_MAX_COLS = 60
README_TREE_MAX_DEPTH = 2
# The H2 skeleton of a strict README: any subset, this order, no repeats.
README_H2_ORDER = (
    "개요",
    "빠른 시작",
    "패키지 구성",
    "인터페이스",
    "설정",
    "디렉토리",
    "문제 해결",
    "더 읽기",
    "의존성",
)
README_REFS_H2 = "더 읽기"  # the one section that may carry issue numbers
TREE_MARKER_RE = re.compile(r"[├└]──")
README_ISSUE_NUMBER_RE = re.compile(r"(?<![\w&/#])#\d{2,5}\b")
MD_LINK_DEST_RE = re.compile(r"(?<=\]\()[^)\s]*")

# D15: a README section cited by name.  README_REF_TARGET_RE is the one place that
# says which files a named-section ref may point at -- widen it (to docs/*.md, ...)
# when issue #849 extends this beyond READMEs.
README_REF_TARGET_RE = re.compile(r"(?:^|/)README\.md$")
# `<path>README(.md)?` (path optional, optional closing backtick) then `§<text>`,
# a straight-quoted "<text>" or a curly-quoted “<text>”.  A straight quote must be
# separated from README by whitespace: `"README.md"` closing a string literal is
# not an opening quote.
README_NAMED_REF_RE = re.compile(
    r"(?<![\w.-])(?P<path>(?:[\w.-]+/)*)README(?P<ext>\.md)?`?"
    r"(?:(?=\s*§\s*(?P<sec>[^\n]*))\s*§|\s+\"(?P<q1>[^\"\s][^\"]*)\"|\s*“(?P<q2>[^”]+)”)"
)
README_LINK_REF_RE = re.compile(
    r"\[(?:[^\]\\]|\\.)*\]\((?P<path>[^)\s#]*README\.md)\)\s*§\s*(?P<sec>[^\n]*)"
)
README_WORD_BEFORE_RE = re.compile(r"`?([\w.-]+)`?\s+$")

# D13: a section ref that names its target constitution.  Accepts the plain
# form ("AGENTS.md §6.5"), the link form ("[AGENTS.md](../AGENTS.md) §6.5") and the
# code-span form ("`AGENTS.md` §6.5") -- the last one hid two stale refs from the
# first version of this check.
SECTION_TARGET_RE = re.compile(r"(CLAUDE|AGENTS)\.md`?(?:\]\([^)]*\))?\s*§(\d+(?:\.\d+)?)")
# A bare "§N" / "§N.M" -- only resolved inside a constitution, against itself.
BARE_SECTION_RE = re.compile(r"§(\d+(?:\.\d+)?)")
# "## 1. Snapshot" / "### 9.1 colcon CWD" / "## 5.5 Inferential" -> "1", "9.1", "5.5"
NUMBERED_HEADING_RE = re.compile(r"^#{2,3}\s+(\d+(?:\.\d+)?)\.?\s", re.M)
# Text files outside the markdown corpus that cite constitution sections, chosen
# by type rather than by directory.  The first cut scanned .claude/hooks and
# repo_scripts only, and six refs in rtc_* sources and .clang-tidy (section 10
# of CLAUDE.md, by number) outlived the renumbering that emptied CLAUDE.md of
# numbered sections while the corpus scan reported clean.  Only D13 runs on these.
SECTION_REF_EXTRA_EXTS = (
    ".sh",
    ".py",
    ".yaml",
    ".yml",
    ".hpp",
    ".cpp",
    ".h",
    ".cc",
    ".cmake",
    ".xml",
)
SECTION_REF_EXTRA_NAMES = frozenset({".clang-tidy", ".clang-format", "CMakeLists.txt"})


def is_section_ref_source(rel: str) -> bool:
    """True for a non-markdown file whose comments D13 resolves."""
    return rel.endswith(SECTION_REF_EXTRA_EXTS) or Path(rel).name in SECTION_REF_EXTRA_NAMES


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


def scanned_files(root: Path) -> list[str]:
    """The files a scan sees: tracked or untracked-but-not-ignored, minus deleted.

    Tracked files alone missed every document written in the change being
    checked -- a new page is untracked until it is committed, so the full scan
    reported a clean tree while fourteen new pages went unchecked (#869).  CI
    checks out a clean tree, where this is the tracked set.  A tracked file
    deleted in the working tree is gone: a citation of it is not satisfied.
    """
    out = subprocess.run(
        ["git", "-C", str(root), "-c", "core.quotePath=false"]
        + ["ls-files", "-z", "--cached", "--others", "--exclude-standard"],
        capture_output=True,
        text=True,
        check=True,
    ).stdout
    seen = dict.fromkeys(f for f in out.split("\0") if f)
    return [f for f in seen if (root / f).is_file()]


class Repo:
    def __init__(self, root: Path):
        self.root = root
        self.files = scanned_files(root)
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
        self._budgets: dict[str, tuple[int, int]] | None = None
        # D14: (budgets, strict paths) of README_BUDGET_FILE, loaded on first use.
        self._readme: tuple[dict[str, tuple[int, int]], set[str]] | None = None
        # D15: normalised section names per README, keyed by repo-relative path.
        self._names_cache: dict[str, set[str]] = {}

    def readme_budgets(self) -> tuple[dict[str, tuple[int, int]], set[str]]:
        """D14 per-README byte budgets and the set of ``strict`` entries."""
        if self._readme is None:
            try:
                text = (self.root / README_BUDGET_FILE).read_text(encoding="utf-8")
            except OSError:
                text = ""
            self._readme = (parse_budgets(text), parse_strict(text))
        return self._readme

    def has_readme_budget_file(self) -> bool:
        """Same guard as :meth:`has_budget_file`, for the README budget file."""
        return (self.root / README_BUDGET_FILE).is_file()

    def readme_section_names(self, rel: str) -> set[str]:
        """D15: the names a ref to README ``rel`` may use (cached)."""
        if rel not in self._names_cache:
            try:
                text = (self.root / rel).read_text(encoding="utf-8")
            except OSError:
                text = ""
            self._names_cache[rel] = section_names(text)
        return self._names_cache[rel]

    def budgets(self) -> dict[str, tuple[int, int]]:
        """D12 per-file byte budgets: ``path -> (bytes, line in the budget file)``."""
        if self._budgets is None:
            try:
                text = (self.root / SIZE_BUDGET_FILE).read_text(encoding="utf-8")
            except OSError:
                text = ""
            self._budgets = parse_budgets(text)
        return self._budgets

    def has_budget_file(self) -> bool:
        """Does this repository declare budgets at all?

        The hook's own test suite drives this validator inside throwaway
        repositories that have an ``agent_docs/`` and no budget file; calling
        every document there "unbudgeted" would fail fixtures about unrelated
        gates.  The price is that deleting the file would switch the per-file
        gate off -- which ``--self-test`` (run by CI on every PR) turns red, by
        pinning the live entries to the tracked constitutional documents.
        """
        return (self.root / SIZE_BUDGET_FILE).is_file()

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
                or is_section_ref_source(f)
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
    if rel.startswith(".github/workflows/"):
        return check_workflow(repo, rel, text)
    # Sources / scripts / configs: only the section refs (D13, D15).
    return [
        *check_section_resolution(repo, rel, text, markdown=False),
        *check_readme_section_refs(repo, rel, text, markdown=False),
    ]


def check_file_section_refs(repo: Repo, rel: str) -> list[Finding]:
    """D13 alone, for ``--section-refs``."""
    try:
        text = (repo.root / rel).read_text(encoding="utf-8")
    except OSError as exc:
        return [Finding(rel, 1, "D0", f"unreadable: {exc}")]
    return check_section_resolution(repo, rel, text, markdown=rel.endswith(".md"))


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
    findings.extend(check_section_refs(rel, text))
    findings.extend(check_section_resolution(repo, rel, text))
    findings.extend(check_rule_ids(repo, rel, text))
    findings.extend(check_constitution_size(rel, text))
    findings.extend(check_line_shape(rel, text))
    findings.extend(check_size_budget(rel, text, repo.budgets()))
    findings.extend(check_readme(repo, rel, text))
    findings.extend(check_readme_section_refs(repo, rel, text))
    if rel in COUNT_SCOPED_DOCS:
        findings.extend(check_package_count(rel, text, repo.package_count(), allowed))
    return findings


def numbered_sections(text: str) -> set[str]:
    return set(NUMBERED_HEADING_RE.findall(text))


_SECTION_CACHE: dict[tuple[Path, str], set[str]] = {}


def target_sections(repo: Repo, target: str) -> set[str]:
    key = (repo.root, target)
    if key not in _SECTION_CACHE:
        try:
            _SECTION_CACHE[key] = numbered_sections(
                (repo.root / target).read_text(encoding="utf-8")
            )
        except OSError:
            _SECTION_CACHE[key] = set()
    return _SECTION_CACHE[key]


def check_constitution_size(rel: str, text: str) -> list[Finding]:
    """D12 -- the constitutions stay inside the line-count and byte ceilings."""
    findings: list[Finding] = []
    if rel not in CONSTITUTION_DOCS:
        return findings
    lines = text.splitlines()
    if len(lines) > CONSTITUTION_MAX_LINES:
        findings.append(
            Finding(
                rel,
                CONSTITUTION_MAX_LINES + 1,
                "D12",
                f"{len(lines)} lines > {CONSTITUTION_MAX_LINES} -- move detail to the "
                "owning agent_docs file or a path-scoped rule; the constitution is a map",
            )
        )
    nbytes = len(text.encode("utf-8"))
    if nbytes > CONSTITUTION_MAX_BYTES:
        findings.append(
            Finding(
                rel,
                1,
                "D12",
                f"{nbytes} bytes > {CONSTITUTION_MAX_BYTES} -- the line cap is being met "
                "by longer lines; cut rationale/history, keep rules + pointers",
            )
        )
    return findings


def check_line_shape(rel: str, text: str) -> list[Finding]:
    """D12 -- a rule document's prose lines and table cells stay citeable.

    One loop for both scopes, which differ only on tables: a constitution's
    table rows are exempt outright, while under LINE_CAP_DIRS the row is free
    and each cell is capped -- the failure that cap exists for is one cell
    swallowing a section.
    """
    findings: list[Finding] = []
    cap_cells = rel.endswith(".md") and rel.startswith(LINE_CAP_DIRS)
    if not (cap_cells or rel in CONSTITUTION_DOCS):
        return findings
    allowed = suppressions(text)
    for lineno, (raw, in_fence) in enumerate(iter_lines_with_fence_state(text), 1):
        if in_fence or "D12" in allowed.get(lineno, frozenset()):
            continue
        if raw.lstrip().startswith("|"):
            if not cap_cells:
                continue
            longest = max(len(cell.strip()) for cell in TABLE_CELL_SPLIT_RE.split(raw))
            if longest > MAX_CELL_CHARS:
                findings.append(
                    Finding(
                        rel,
                        lineno,
                        "D12",
                        f"{longest}-char table cell > {MAX_CELL_CHARS} -- a cell holds one "
                        "rule; lift the rest into bullets under the table",
                    )
                )
        elif len(raw) > CONSTITUTION_MAX_LINE_CHARS:
            findings.append(
                Finding(
                    rel,
                    lineno,
                    "D12",
                    f"{len(raw)}-char line > {CONSTITUTION_MAX_LINE_CHARS} -- split into "
                    "bullets or move the detail to the document that owns it",
                )
            )
    return findings


def is_budgeted(rel: str) -> bool:
    """Is ``rel`` one of the documents D12 budgets per file (corpus and skills)?"""
    return rel in CONSTITUTION_DOCS or (rel.endswith(".md") and rel.startswith(SIZE_BUDGET_DIRS))


def parse_budgets(text: str) -> dict[str, tuple[int, int]]:
    """``path: bytes`` lines of SIZE_BUDGET_FILE -> ``path -> (bytes, lineno)``."""
    out: dict[str, tuple[int, int]] = {}
    for lineno, raw in enumerate(text.splitlines(), 1):
        m = SIZE_BUDGET_LINE_RE.match(raw)
        if m:
            out[m.group(1)] = (int(m.group(2)), lineno)
    return out


def parse_strict(text: str) -> set[str]:
    """The paths of a budget file whose entry is marked ``strict``."""
    out: set[str] = set()
    for raw in text.splitlines():
        m = SIZE_BUDGET_LINE_RE.match(raw)
        if m and m.group(3):
            out.add(m.group(1))
    return out


def check_size_budget(
    rel: str,
    text: str,
    budgets: dict[str, tuple[int, int]],
    *,
    code: str = "D12",
    budget_file: str = SIZE_BUDGET_FILE,
    hint: str = "state the rule and point at the document that owns the detail",
) -> list[Finding]:
    """D12 / D14 -- a budgeted document stays inside its own byte budget.

    The message keeps the ``<n> bytes > <cap>`` shape of the constitution cap on
    purpose: the Stop hook narrows doc findings to the lines a change added and
    lets exactly that shape through, since a whole-file budget is never blown
    on the line it is reported at.
    """
    if rel not in budgets:
        return []
    nbytes = len(text.encode("utf-8"))
    budget = budgets[rel][0]
    if nbytes <= budget:
        return []
    return [
        Finding(
            rel,
            1,
            code,
            f"{nbytes} bytes > {budget} -- over this file's budget in {budget_file}; "
            f"{hint} "
            "(a budget goes down as its file shrinks, not up to fit an edit; a rule or section "
            "that really grew is raised on its own line with `# raised from N: <why>`)",
        )
    ]


def check_budget_coverage(
    sizes: dict[str, int],
    budgets: dict[str, tuple[int, int]],
    existing: set[str] | None,
    *,
    code: str = "D12",
    budget_file: str = SIZE_BUDGET_FILE,
    noun: str = "constitutional document",
) -> list[Finding]:
    """D12 / D14 -- every budgeted document has an entry, and every entry has a document.

    ``sizes`` maps the budgeted documents under scan to their byte size.  Pass
    ``existing`` (the tracked budgeted documents) only for a full-corpus scan:
    a ``--files`` run sees a slice of the corpus and cannot call an entry stale.
    """
    findings = [
        Finding(
            rel,
            1,
            code,
            f"{nbytes} bytes > 0 -- no entry in {budget_file}; a new {noun} "
            "declares its byte budget in the change that adds it",
        )
        for rel, nbytes in sorted(sizes.items())
        if rel not in budgets
    ]
    if existing is not None:
        findings.extend(
            Finding(
                budget_file,
                lineno,
                code,
                f"budget entry for '{path}', which is not a tracked {noun} "
                "-- remove the entry or fix the path",
            )
            for path, (_, lineno) in sorted(budgets.items())
            if path not in existing
        )
    return findings


def check_budget_raises(
    text: str, base_text: str, *, code: str = "D12", budget_file: str = SIZE_BUDGET_FILE
) -> list[Finding]:
    """D12 / D14 -- a budget above its value in the base revision carries a raise note.

    An entry the base does not have is a new document declaring its budget, not
    a raise; so is every entry when the base has no budget file at all.
    """
    base = parse_budgets(base_text)
    lines = text.splitlines()
    findings: list[Finding] = []
    for path, (value, lineno) in sorted(parse_budgets(text).items()):
        was = base.get(path, (value, 0))[0]
        if value <= was:
            continue
        note = BUDGET_RAISE_NOTE_RE.search(lines[lineno - 1])
        if note and int(note.group(1)) == was:
            continue
        findings.append(
            Finding(
                budget_file,
                lineno,
                code,
                f"budget for '{path}' raised {was} -> {value} -- a budget goes down, not up "
                "to fit an edit; move the content to the document that owns it, or if a "
                f"rule was added, say so on this line: '# raised from {was}: <why>'",
            )
        )
    return findings


def base_budget_text(repo: Repo, rev: str, budget_file: str = SIZE_BUDGET_FILE) -> str | None:
    """The budget file as of ``rev``: "" when it has none, None when ``rev`` is not a commit.

    The two must not collapse: a base that never had the file has nothing to
    ratchet against, while a revision that does not resolve (a shallow clone
    that never fetched it, a typo) would otherwise turn the gate off silently.
    """

    def git(*args: str) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            ["git", "-C", str(repo.root), *args], capture_output=True, text=True, check=False
        )

    if git("cat-file", "-e", f"{rev}^{{commit}}").returncode != 0:
        return None
    shown = git("show", f"{rev}:{budget_file}")
    return shown.stdout if shown.returncode == 0 else ""


def is_readme(rel: str) -> bool:
    """Is ``rel`` a file D14 budgets -- basename exactly ``README.md``?"""
    return Path(rel).name == README_NAME


def check_readme(repo: Repo, rel: str, text: str) -> list[Finding]:
    """D14 -- a tracked README stays inside its byte budget; a ``strict`` one keeps its shape."""
    if not is_readme(rel):
        return []
    budgets, strict = repo.readme_budgets()
    findings = check_size_budget(
        rel,
        text,
        budgets,
        code="D14",
        budget_file=README_BUDGET_FILE,
        hint="move the detail to the document that owns it (docs/, a header comment, an issue)",
    )
    if rel in strict:
        findings.extend(check_readme_shape(rel, text))
    return findings


def check_readme_shape(rel: str, text: str) -> list[Finding]:
    """D14 -- the shape contract of a ``strict`` README.

    Cell and prose caps, file-tree width and depth, the heading skeleton, and no
    issue numbers in the body.  Fenced blocks are exempt from everything except
    the file-tree rule, which is about them.  ``allow D14`` covers a line (for a
    tree block: the line the marker sits on and the next).
    """
    findings: list[Finding] = []
    allowed = suppressions(text)

    def ok(lineno: int) -> bool:
        return "D14" in allowed.get(lineno, frozenset())

    def add(lineno: int, msg: str) -> None:
        findings.append(Finding(rel, lineno, "D14", msg))

    headings: list[tuple[int, int, str]] = []  # (lineno, level, text)
    block: list[tuple[int, str]] = []  # the fenced block being collected
    in_refs = False  # inside the `## 더 읽기` section
    for lineno, (raw, in_fence) in enumerate(iter_lines_with_fence_state(text), 1):
        if in_fence:
            block.append((lineno, raw))
            continue
        if block:
            findings.extend(_check_tree_block(rel, block, allowed))
            block = []
        m = HEADING_RE.match(raw)
        if m:
            level = len(m.group(1))
            headings.append((lineno, level, raw.lstrip("#").strip()))
            if level <= 2:
                in_refs = level == 2 and headings[-1][2] == README_REFS_H2
        if ok(lineno):
            continue
        if raw.lstrip().startswith("|"):
            longest = max(len(cell.strip()) for cell in TABLE_CELL_SPLIT_RE.split(raw))
            if longest > README_MAX_CELL_CHARS:
                add(
                    lineno,
                    f"{longest}-char table cell > {README_MAX_CELL_CHARS} -- a cell holds one "
                    "fact; lift the rest into prose or bullets under the table",
                )
        elif len(raw) > README_MAX_PROSE_CHARS:
            add(
                lineno,
                f"{len(raw)}-char line > {README_MAX_PROSE_CHARS} -- split into bullets or "
                "move the detail to the document that owns it",
            )
        if not m and not in_refs:
            body = MD_LINK_DEST_RE.sub("", mask_code_spans(raw))
            for im in README_ISSUE_NUMBER_RE.finditer(body):
                add(
                    lineno,
                    f"issue number '{im.group(0)}' in the body -- issue numbers go in "
                    f"'## {README_REFS_H2}' or git log",
                )
    if block:
        findings.extend(_check_tree_block(rel, block, allowed))

    h1s = [h for h in headings if h[1] == 1]
    if len(h1s) != 1 or (headings and headings[0][1] != 1):
        add(
            h1s[1][0] if len(h1s) > 1 else 1,
            f"{len(h1s)} H1 headings (first heading is "
            f"{f'H{headings[0][1]}' if headings else 'absent'}) -- a README has exactly one "
            "H1 and it is the first heading",
        )
    last = -1
    seen: set[str] = set()
    for lineno, level, title in headings:
        if level != 2 or ok(lineno):
            continue
        if title not in README_H2_ORDER:
            add(
                lineno,
                f"H2 '{title}' is not in the README skeleton ({' / '.join(README_H2_ORDER)}) "
                "-- make it an H3 under a skeleton section, or move it to the doc that owns it",
            )
        elif title in seen:
            add(lineno, f"duplicate H2 '{title}' -- merge it into the first one")
        elif README_H2_ORDER.index(title) < last:
            add(
                lineno,
                f"H2 '{title}' is out of order -- the skeleton order is {' / '.join(README_H2_ORDER)}",
            )
        if title in README_H2_ORDER:
            seen.add(title)
            last = max(last, README_H2_ORDER.index(title))
    return findings


def _check_tree_block(
    rel: str, block: list[tuple[int, str]], allowed: dict[int, set[str]]
) -> list[Finding]:
    """D14 -- a fenced file tree is at most 60 columns wide and 2 levels deep.

    One finding per rule per block (the first offending line, plus a count): an
    annotated tree that is wrong is wrong on hundreds of lines.
    """
    if not any(TREE_MARKER_RE.search(raw) for _, raw in block):
        return []
    wide = [n for n, raw in block if len(raw) > README_TREE_MAX_COLS]
    deep = [
        n
        for n, raw in block
        if (mk := TREE_MARKER_RE.search(raw)) and mk.start() // 4 + 1 > README_TREE_MAX_DEPTH
    ]
    out: list[Finding] = []
    for lines, msg in (
        (
            wide,
            f"file tree line longer than {README_TREE_MAX_COLS} columns -- keep the tree to "
            "directories and move per-file notes into the file's header comment",
        ),
        (
            deep,
            f"file tree deeper than {README_TREE_MAX_DEPTH} levels -- show the top "
            f"{README_TREE_MAX_DEPTH} levels and point at the directory for the rest",
        ),
    ):
        lines = [n for n in lines if "D14" not in allowed.get(n, frozenset())]
        if lines:
            more = f" (+{len(lines) - 1} more lines)" if len(lines) > 1 else ""
            out.append(Finding(rel, lines[0], "D14", msg + more))
    return out


# --------------------------------------------------------------------------
# D15 -- README sections cited by name
# --------------------------------------------------------------------------


def norm_section_name(text: str) -> str:
    """Normalise a heading / bold span / cited name so they compare equal.

    Backticks and ``*`` dropped, surrounding quotes and whitespace stripped,
    whitespace collapsed, case folded.
    """
    text = text.replace("`", "").replace("*", "")
    text = text.strip().strip("\"'“”‘’").strip()
    return " ".join(text.split()).casefold()


def section_names(text: str) -> set[str]:
    """Every name a ref may use for a document: its headings (H1-H6) and bold spans."""
    names: set[str] = set()
    for raw, in_fence in iter_lines_with_fence_state(text):
        if in_fence:
            continue
        m = HEADING_RE.match(raw)
        if m:
            names.add(norm_section_name(m.group(2)))
        names.update(norm_section_name(b) for b in BOLD_SPAN_RE.findall(raw))
    # A heading often carries a gloss after its title ("흔한 실패와 감별 (colcon
    # cwd · ...)", "Scene overlay (`sim_overlay:=`) -- ...") and a ref cites the
    # title alone, so the title before the first " (" / dash is a name as well.
    names.update(HEADING_GLOSS_RE.split(n, maxsplit=1)[0] for n in list(names))
    names.discard("")
    return names


HEADING_GLOSS_RE = re.compile(r"\s+(?:\(|[\u2014\u2013])")


BOLD_SPAN_RE = re.compile(r"\*\*(.+?)\*\*")


def _resolve_readme(repo: Repo, rel: str, path: str) -> str | None:
    """The tracked README that a ref in ``rel`` means by ``path`` (``""`` = bare README).

    Relative to the citing file, then the repo root, then a unique suffix among the
    tracked READMEs; a bare README is the nearest ancestor.  ``None`` = unresolvable
    (D1's business, not D15's).
    """
    if path in ("", README_NAME):
        d = Path(rel).parent
        while True:
            cand = (d / README_NAME).as_posix().removeprefix("./")
            if cand in repo.fileset:
                return cand
            if d.as_posix() in (".", ""):
                return None
            d = d.parent
    if not path.endswith(".md"):
        path += ".md"
    for cand in ((Path(rel).parent / path).as_posix(), path):
        norm = normalise(cand)
        if norm and norm in repo.fileset and README_REF_TARGET_RE.search(norm):
            return norm
    hits = [f for f in repo.files if (f == path or f.endswith("/" + path)) and is_readme(f)]
    return hits[0] if len(hits) == 1 else None


def check_readme_section_refs(
    repo: Repo, rel: str, text: str, *, markdown: bool = True
) -> list[Finding]:
    """D15 -- a README section cited by name names a heading or bold span that exists."""
    findings: list[Finding] = []
    allowed = suppressions(text)
    lines = (
        iter_lines_with_fence_state(text) if markdown else ((r, False) for r in text.splitlines())
    )
    for lineno, (raw, in_fence) in enumerate(lines, 1):
        if in_fence or "D15" in allowed.get(lineno, frozenset()) or "README" not in raw:
            continue
        # (readme path as cited, cited text) per ref on the line
        refs: list[tuple[str, str]] = []
        links = [
            (lm.start(), lm.end(), lm.group(1).split("#")[0]) for lm in MD_LINK_RE.finditer(raw)
        ]
        for m in README_NAMED_REF_RE.finditer(raw):
            path = m.group("path") + README_NAME if m.group("path") else ""
            # Inside a link label the link's own target says which README is meant
            # (a label that names README and quotes a title, linking to the root one).
            label_target = next(
                (
                    t
                    for a, b, t in links
                    if a < m.start() < b and README_REF_TARGET_RE.search(t) and not m.group("path")
                ),
                None,
            )
            if label_target:
                path = label_target
            elif not m.group("path"):
                # A bare README preceded by a directory name (a package, say) is
                # that directory's README.
                w = README_WORD_BEFORE_RE.search(raw[: m.start()])
                if w and _resolve_readme(repo, rel, f"{w.group(1)}/README.md"):
                    path = f"{w.group(1)}/README.md"
            cited = (
                m.group("sec") if m.group("sec") is not None else m.group("q1") or m.group("q2")
            )
            refs.append((path, cited.split("|")[0] if m.group("sec") is not None else cited))
        for m in README_LINK_REF_RE.finditer(raw):
            refs.append((m.group("path"), m.group("sec").split("|")[0]))
        for path, cited in refs:
            target = _resolve_readme(repo, rel, path)
            wanted = norm_section_name(cited)
            if target is None or not wanted or wanted[0].isdigit():
                continue
            # `§` form: some name is a prefix of the rest of the line (the name's
            # extent is unknown).  Quoted form: equals a name or extends one.  Both
            # reduce to the same prefix test.
            if not any(wanted.startswith(n) for n in repo.readme_section_names(target)):
                shown = " ".join(cited.split())[:60]
                findings.append(
                    Finding(
                        rel,
                        lineno,
                        "D15",
                        f"'{target}' has no heading or bold span matching '{shown}' -- keep the "
                        "section, or re-point every citing line in the same change and delete "
                        "the heading (no stub headings)",
                    )
                )
    return findings


def check_section_resolution(
    repo: Repo, rel: str, text: str, *, markdown: bool = True
) -> list[Finding]:
    """D13 -- a CLAUDE.md / AGENTS.md section ref names a heading that exists."""
    findings: list[Finding] = []
    allowed = suppressions(text)
    own = numbered_sections(text) if rel in CONSTITUTION_DOCS else None
    if markdown:
        lines = iter_lines_with_fence_state(text)
    else:
        lines = ((raw, False) for raw in text.splitlines())
    for lineno, (raw, in_fence) in enumerate(lines, 1):
        if in_fence or "D13" in allowed.get(lineno, frozenset()):
            continue
        consumed: list[tuple[int, int]] = []
        for m in SECTION_TARGET_RE.finditer(raw):
            consumed.append(m.span())
            target = f"{m.group(1)}.md"
            sec = m.group(2)
            have = own if (own is not None and target == rel) else target_sections(repo, target)
            if sec not in have:
                findings.append(
                    Finding(
                        rel,
                        lineno,
                        "D13",
                        f"'{target} §{sec}' names a section {target} does not have "
                        "(renumbered or removed) -- repoint the reference",
                    )
                )
        if own is None:
            continue
        for m in BARE_SECTION_RE.finditer(raw):
            if any(a <= m.start() < b for a, b in consumed):
                continue
            # "handoff.md §5" / "compliance §6.5" belong to another namespace.
            if SECTION_REF_QUALIFIED_RE.search(raw[: m.start()]):
                continue
            if m.group(1) not in own:
                findings.append(
                    Finding(
                        rel,
                        lineno,
                        "D13",
                        f"'§{m.group(1)}' has no matching numbered heading in {rel}",
                    )
                )
    return findings


def check_section_refs(rel: str, text: str) -> list[Finding]:
    """D10 -- numeric section refs must name their namespace."""
    findings: list[Finding] = []
    if not rel.startswith(SECTION_REF_SCOPED_DOCS):
        return findings
    # A document that numbers its own headings ("## 6. Escalation Triggers") is
    # the owner of those numbers, so "§6.5" inside it is a self-reference and
    # needs no prefix.  The files that carry the ambiguity are the ones using
    # named headings -- there, every numeric ref is necessarily cross-document.
    if SELF_NUMBERED_HEADING_RE.search(text):
        return findings
    allowed = suppressions(text)
    for lineno, (raw, in_fence) in enumerate(iter_lines_with_fence_state(text), 1):
        if in_fence or "D10" in allowed.get(lineno, frozenset()):
            continue
        for m in SECTION_REF_RE.finditer(raw):
            if SECTION_REF_QUALIFIED_RE.search(raw[: m.start()]):
                continue
            findings.append(
                Finding(
                    rel,
                    lineno,
                    "D10",
                    f"bare section ref '{m.group(0)}' -- prefix it with the owning "
                    "document ('AGENTS.md \u00a76.5') or 'compliance ' for the "
                    "normative spec",
                )
            )
    return findings


def check_rule_ids(repo: Repo, rel: str, text: str) -> list[Finding]:
    """D11 -- every referenced rule ID must be defined by its owning file."""
    findings: list[Finding] = []
    allowed = suppressions(text)
    for lineno, (raw, in_fence) in enumerate(iter_lines_with_fence_state(text), 1):
        if in_fence or "D11" in allowed.get(lineno, frozenset()):
            continue
        for m in RULE_ID_RE.finditer(raw):
            rid = m.group(1)
            if rid in RULE_ID_RETIRED:
                continue
            ns = rid.split("-")[0] if "-" in rid else "P"
            owner = RULE_ID_OWNERS.get(ns)
            if owner is None or rel == owner:
                continue
            if not rule_id_defined(repo, owner, rid):
                findings.append(
                    Finding(
                        rel,
                        lineno,
                        "D11",
                        f"rule ID '{rid}' is not defined in its owner {owner}",
                    )
                )
    return findings


@functools.cache
def rule_id_defined(repo: Repo, owner: str, rid: str) -> bool:
    """True when *owner* defines *rid* (table row, heading, or bold label)."""
    path = repo.root / owner
    try:
        body = path.read_text(encoding="utf-8")
    except OSError:
        # Owner missing is a D1 concern; do not double-report it as D11.
        return True
    for pat in (
        re.compile(r"^\|\s*" + re.escape(rid) + r"\s*\|", re.M),
        re.compile(r"^#{2,4}\s+" + re.escape(rid) + r"\b", re.M),
        re.compile(r"\*\*" + re.escape(rid) + r"\b"),
    ):
        if pat.search(body):
            return True
    return False


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
            "package-relative resolution plus include shorthand). A file deleted on "
            "purpose is cited by name or as `git show <commit>:<path>`, not as a path",
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


# D13 over non-markdown files: (name, rel, body, codes expected).
SOURCE_REF_FIXTURES: list[tuple[str, str, str, list[str]]] = [
    (
        "C++ comment with a dangling ref",
        "rtc_x/include/x.hpp",
        "// per CLAUDE.md \u00a710\n",
        ["D13"],
    ),
    ("C++ comment with a live ref", "rtc_x/src/x.cpp", "// per AGENTS.md \u00a710\n", []),
    (
        "tool config with a dangling ref",
        ".clang-tidy",
        "# see CLAUDE.md \u00a710 Style\n",
        ["D13"],
    ),
]
# Real tracked files of each kind that carried a constitution ref when D13 was
# first widened; each must stay in the scanned set.
SOURCE_REF_CORPUS_MEMBERS = (
    ".clang-tidy",
    "rtc_base/include/rtc_base/types/types.hpp",
    "integrated_bringup/src/controllers/task/controller.cpp",
    "rtc_mujoco_sim/config/mujoco_default.yaml",
)


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
    ("D12 constitution over the line cap", "CLAUDE.md", "# x\n" + "- a\n" * 200, ["D12"]),
    (
        "D12 constitution over the byte cap",
        "AGENTS.md",
        "# x\n" + ("\uac00" * 100 + "\n") * 70,
        ["D12"],
    ),
    ("D12 over-long prose line", "CLAUDE.md", "# x\n" + "a" * 501 + "\n", ["D12"]),
    ("D12 table row is exempt", "CLAUDE.md", "# x\n| " + "a" * 600 + " |\n", []),
    ("D12 fenced line is exempt", "CLAUDE.md", "# x\n```\n" + "a" * 600 + "\n```\n", []),
    # The prose-line cap covers agent_docs/ as well (this fixture asserted the
    # opposite while the cap was constitution-only), and stops there.
    (
        "D12 prose-line cap covers agent_docs",
        "agent_docs/f.md",
        "# x\n" + "a" * 501 + "\n",
        ["D12"],
    ),
    ("D12 line shape stops at agent_docs", "docs/f.md", "# x\n" + "a" * 501 + "\n", []),
    ("D12 line shape leaves rules alone", ".claude/rules/f.md", "# x\n" + "a" * 501 + "\n", []),
    ("D12 prose line at the cap", "agent_docs/f.md", "# x\n" + "a" * 500 + "\n", []),
    ("D12 over-long table cell", "agent_docs/f.md", "| a | " + "b" * 501 + " |\n", ["D12"]),
    ("D12 table cell at the cap", "agent_docs/f.md", "| a | " + "b" * 500 + " |\n", []),
    (
        "D12 a long row of short cells is fine",
        "agent_docs/f.md",
        "|" + (" " + "c" * 200 + " |") * 4 + "\n",
        [],
    ),
    (
        "D12 an escaped pipe does not split a cell",
        "agent_docs/f.md",
        "| " + "d" * 300 + " \\| " + "e" * 300 + " |\n",
        ["D12"],
    ),
    (
        "D12 fenced line in agent_docs is exempt",
        "agent_docs/f.md",
        "```\n" + "a" * 600 + "\n```\n",
        [],
    ),
    (
        "D12 allow marker covers an agent_docs line",
        "agent_docs/f.md",
        "<!-- validate-docs: allow D12 -->\n" + "a" * 501 + "\n",
        [],
    ),
    ("D13 dangling section ref", "agent_docs/f.md", "see CLAUDE.md \u00a799.9\n", ["D13"]),
    (
        "D13 dangling ref in link form",
        "agent_docs/f.md",
        "see [AGENTS.md](../AGENTS.md) \u00a799\n",
        ["D13"],
    ),
    (
        "D13 live refs resolve",
        "agent_docs/f.md",
        "see AGENTS.md \u00a71 and AGENTS.md \u00a79.1\n",
        [],
    ),
    ("D13 dangling ref in code-span form", "docs/f.md", "rule of `AGENTS.md` \u00a799\n", ["D13"]),
    ("D13 live ref in code-span form", "docs/f.md", "rule of `AGENTS.md` \u00a710\n", []),
    (
        "D10 code-span document name qualifies",
        "agent_docs/f.md",
        "see `AGENTS.md` \u00a76.5\n",
        [],
    ),
    (
        "D13 bare self-ref inside a constitution",
        "CLAUDE.md",
        "# x\n\n## 1. A\n\nsee \u00a71 and \u00a77\n",
        ["D13"],
    ),
    (
        "D13 other-namespace bare ref is not a self-ref",
        "CLAUDE.md",
        "# x\n\n## 1. A\n\nsee handoff.md \u00a75 and compliance \u00a76.5\n",
        [],
    ),
    (
        "D13 suppressed",
        "agent_docs/f.md",
        "<!-- validate-docs: allow D13 -->\nold text said CLAUDE.md \u00a799.9\n",
        [],
    ),
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
    # The companions left agent_docs/ with their content; the checks went along.
    (
        "D4 covers the constitution's companions",
        "docs/reference/f.md",
        "see rt_controller_node.cpp:120\n",
        ["D4"],
    ),
    ("D4 covers a companion file", "docs/testing.md", "see rt_controller_node.cpp:120\n", ["D4"]),
    (
        "D8 covers the anti-pattern case book",
        "docs/reference/anti-patterns.md",
        "| RT-1 | `grep -rnE 'RCLCPP_(INFO|WARN)' src/` |\n",
        ["D8"],
    ),
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
    # D10 severance guard: a bare compliance-spec ref in a named-heading doc.
    (
        "D10 bare section ref",
        "agent_docs/f.md",
        "특이점 처리는 §6.5 를 따른다.\n",
        ["D10"],
    ),
    # Qualified three ways -- none of these may fire, or the check is unusable.
    (
        "D10 qualified refs are silent",
        "agent_docs/f.md",
        "compliance §6.5 와 [AGENTS.md](../AGENTS.md) §6.5 와 AGENTS.md §9.1.\n",
        [],
    ),
    # A doc that numbers its own headings owns those numbers.
    (
        "D10 self-reference in a numbered doc",
        "agent_docs/f.md",
        "## 6. Escalation\n\n자세한 것은 §6.5 참조.\n",
        [],
    ),
    # Out of the constitution corpus entirely.
    (
        "D10 out of scope",
        "rtc_controllers/docs/spec.md",
        "특이점 처리는 §6.5 를 따른다.\n",
        [],
    ),
    (
        "D10 covers the constitution's companions",
        "docs/modification-procedures.md",
        "특이점 처리는 §6.5 를 따른다.\n",
        ["D10"],
    ),
    ("D10 stops at the companions", "docs/tracing.md", "특이점 처리는 §6.5 를 따른다.\n", []),
    # D11 severance guard: P9 is never a real principle, so this stays red as
    # long as check_rule_ids is wired into check_markdown.
    (
        "D11 undefined rule ID",
        "agent_docs/f.md",
        "새 utility 는 P9 를 따른다.\n",
        ["D11"],
    ),
    # One undefined ID per namespace whose owner is a different file.  A stale
    # owner path makes rule_id_defined() answer True for everything (a missing
    # owner is left to D1), so the "defined IDs are silent" fixture below cannot
    # tell a working check from one whose owner file has moved away.
    (
        "D11 undefined AP ID (owner path is live)",
        "agent_docs/f.md",
        "AP-PROC-99 는 없는 패턴이다.\n",
        ["D11"],
    ),
    (
        "D11 undefined invariant ID (owner path is live)",
        "agent_docs/f.md",
        "RT-99 는 없는 규칙이다.\n",
        ["D11"],
    ),
    # ...and a defined one must stay silent, or every doc lights up.
    (
        "D11 defined rule IDs are silent",
        "agent_docs/f.md",
        "P5 와 ARCH-1 과 NUM-5 와 AP-RT-1 은 실재한다.\n",
        [],
    ),
    # Retired IDs are cited as history, not as live rules.
    (
        "D11 retired ID is silent",
        "agent_docs/f.md",
        "RT-7 은 은퇴했다.\n",
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

# D12 per-file budgets.  (name, rel, body, budget-file text, want codes) through
# check_size_budget; the budget text is parsed by parse_budgets, so the cases
# exercise the line format as well as the comparison.
BUDGET_CASES: list[tuple[str, str, str, str, list[str]]] = [
    ("at the budget", "agent_docs/f.md", "a" * 10, "agent_docs/f.md: 10\n", []),
    ("one byte over", "agent_docs/f.md", "a" * 11, "agent_docs/f.md: 10\n", ["D12"]),
    # Budgets are bytes, not characters: 4 Hangul syllables are 12 bytes.
    ("bytes, not characters", "agent_docs/f.md", "가" * 4, "agent_docs/f.md: 10\n", ["D12"]),
    ("another file's entry", "agent_docs/f.md", "a" * 99, "agent_docs/g.md: 10\n", []),
    ("trailing comment", ".claude/rules/r.md", "a" * 11, ".claude/rules/r.md: 10  # x\n", ["D12"]),
    # A commented-out entry is no entry: it must not gate, and coverage reports it.
    ("commented-out entry", "agent_docs/f.md", "a" * 99, "# agent_docs/f.md: 10\n", []),
]
# (name, sizes under scan, budget-file text, tracked budgeted docs or None, want
# "path:code" pairs) through check_budget_coverage.
BUDGET_COVERAGE_CASES: list[
    tuple[str, dict[str, int], str, set[str] | None, list[tuple[str, str]]]
] = [
    ("covered", {"agent_docs/f.md": 5}, "agent_docs/f.md: 10\n", {"agent_docs/f.md"}, []),
    (
        "new document without an entry",
        {"agent_docs/new.md": 5},
        "agent_docs/f.md: 10\n",
        None,
        [("agent_docs/new.md", "D12")],
    ),
    (
        "entry whose document is gone",
        {},
        "agent_docs/gone.md: 10\n",
        set(),
        [(SIZE_BUDGET_FILE, "D12")],
    ),
    # A --files run sees a slice of the corpus: it cannot call an entry stale.
    ("stale entry is a full-scan finding only", {}, "agent_docs/gone.md: 10\n", None, []),
]
# (name, budget-file text, base budget-file text, want codes) through
# check_budget_raises.
BUDGET_RAISE_CASES: list[tuple[str, str, str, list[str]]] = [
    ("unchanged", "agent_docs/f.md: 10\n", "agent_docs/f.md: 10\n", []),
    ("lowered", "agent_docs/f.md: 9\n", "agent_docs/f.md: 10\n", []),
    ("raised without a note", "agent_docs/f.md: 11\n", "agent_docs/f.md: 10\n", ["D12"]),
    (
        "raised with its note",
        "agent_docs/f.md: 11  # raised from 10: RT-11 added\n",
        "agent_docs/f.md: 10\n",
        [],
    ),
    # A note from an earlier raise names the value before THAT raise.
    (
        "an earlier raise's note authorises nothing",
        "agent_docs/f.md: 12  # raised from 10: RT-11 added\n",
        "agent_docs/f.md: 11  # raised from 10: RT-11 added\n",
        ["D12"],
    ),
    (
        "a note without a reason",
        "agent_docs/f.md: 11  # raised from 10:\n",
        "agent_docs/f.md: 10\n",
        ["D12"],
    ),
    (
        "any other comment is not a note",
        "agent_docs/f.md: 11  # ok\n",
        "agent_docs/f.md: 10\n",
        ["D12"],
    ),
    # A new document declares its budget; that is not a raise.
    (
        "entry the base lacks",
        "agent_docs/f.md: 10\nagent_docs/g.md: 99\n",
        "agent_docs/f.md: 10\n",
        [],
    ),
    ("base without a budget file", "agent_docs/f.md: 10\n", "", []),
    (
        "only the raised entry is reported",
        "agent_docs/f.md: 11\nagent_docs/g.md: 5\n",
        "agent_docs/f.md: 10\nagent_docs/g.md: 5\n",
        ["D12"],
    ),
]
BUDGET_SCOPE_CASES: list[tuple[str, bool]] = [
    ("AGENTS.md", True),
    ("CLAUDE.md", True),
    ("agent_docs/invariants.md", True),
    (".claude/rules/rt-path.md", True),
    ("README.md", False),
    ("docs/tracing.md", False),
    (".claude/skills/adding-component/SKILL.md", True),
    (".claude/skills/x/references/y.md", True),
    (".claude/skills/x/run.sh", False),
    ("agent_docs/budget.yaml", False),
]

# D14 README shape.  (name, rel, markdown, readme_budget.yaml text, codes) through
# check_markdown.  The budget text carries the `strict` flag, so each case also
# exercises the entry format; the near-miss of every cap sits beside it.
_STRICT = "pkg/README.md: 99999 strict\n"
_LAX = "pkg/README.md: 99999\n"
_TREE3 = "```\npkg/\n├── a\n│   ├── b\n│   │   ├── c\n```\n"


def _readme(*body: str) -> str:
    return "# Title\n\n" + "\n".join(body) + "\n"


README_FIXTURES: list[tuple[str, str, str, str, list[str]]] = [
    (
        "strict skeleton is clean",
        "pkg/README.md",
        _readme("## 개요", "text", "## 설정", "| a | b |", "## 더 읽기", "- 이슈 #123"),
        _STRICT,
        [],
    ),
    ("over the byte budget", "pkg/README.md", "# T\n" + "a" * 20, "pkg/README.md: 10\n", ["D14"]),
    ("at the byte budget", "pkg/README.md", "a" * 10, "pkg/README.md: 10\n", []),
    (
        "bytes, not characters",
        "pkg/README.md",
        "가" * 4,
        "pkg/README.md: 10\n",
        ["D14"],
    ),
    ("only the exact basename", "pkg/MY_README.md", "a" * 600, _STRICT, []),
    ("no entry, no gate", "pkg/README.md", "a" * 600, "", []),
    (
        "shape checks need strict",
        "pkg/README.md",
        _readme("| a | " + "b" * 300 + " |", "a" * 600, "## 임의"),
        _LAX,
        [],
    ),
    (
        "table cell over the cap",
        "pkg/README.md",
        _readme("| a | " + "b" * 201 + " |"),
        _STRICT,
        ["D14"],
    ),
    (
        "table cell at the cap",
        "pkg/README.md",
        _readme("| a | " + "b" * 200 + " |"),
        _STRICT,
        [],
    ),
    ("prose line over the cap", "pkg/README.md", _readme("a" * 501), _STRICT, ["D14"]),
    ("prose line at the cap", "pkg/README.md", _readme("a" * 500), _STRICT, []),
    (
        "a fenced prose line is exempt",
        "pkg/README.md",
        _readme("```bash", "a" * 600, "```"),
        _STRICT,
        [],
    ),
    (
        "file tree line over 60 columns",
        "pkg/README.md",
        _readme("```", "├── " + "a" * 57, "```"),
        _STRICT,
        ["D14"],
    ),
    (
        "file tree line at 60 columns",
        "pkg/README.md",
        _readme("```", "├── " + "a" * 56, "```"),
        _STRICT,
        [],
    ),
    ("file tree three levels deep", "pkg/README.md", _readme(_TREE3), _STRICT, ["D14"]),
    (
        "file tree two levels deep",
        "pkg/README.md",
        _readme("```", "pkg/", "├── a", "│   ├── b", "└── c", "```"),
        _STRICT,
        [],
    ),
    (
        "a wide fence without a tree is exempt",
        "pkg/README.md",
        _readme("```", "x" * 90, "```"),
        _STRICT,
        [],
    ),
    (
        "a long tree is one finding",
        "pkg/README.md",
        _readme("```", *["├── " + "a" * 60] * 5, "```"),
        _STRICT,
        ["D14"],
    ),
    ("two H1s", "pkg/README.md", "# A\n## 개요\n# B\n", _STRICT, ["D14"]),
    ("H1 not first", "pkg/README.md", "## 개요\n# A\n", _STRICT, ["D14"]),
    ("no H1", "pkg/README.md", "## 개요\n", _STRICT, ["D14"]),
    ("H1 inside a fence is not a heading", "pkg/README.md", "# A\n```\n# B\n```\n", _STRICT, []),
    ("H2 outside the skeleton", "pkg/README.md", _readme("## 잡담"), _STRICT, ["D14"]),
    ("duplicate H2", "pkg/README.md", _readme("## 개요", "## 개요"), _STRICT, ["D14"]),
    ("H2 out of order", "pkg/README.md", _readme("## 설정", "## 개요"), _STRICT, ["D14"]),
    (
        "H2 subset in order",
        "pkg/README.md",
        _readme("## 개요", "## 인터페이스", "## 의존성"),
        _STRICT,
        [],
    ),
    (
        "H3 is free",
        "pkg/README.md",
        _readme("## 개요", "### 아무 제목 #123", "#### 더 깊이"),
        _STRICT,
        [],
    ),
    (
        "H2 compared with trailing spaces stripped only",
        "pkg/README.md",
        _readme("## 개요  "),
        _STRICT,
        [],
    ),
    ("issue number in the body", "pkg/README.md", _readme("회귀 (#123) 때문"), _STRICT, ["D14"]),
    (
        "issue number in a table cell",
        "pkg/README.md",
        _readme("| a | #4567 |"),
        _STRICT,
        ["D14"],
    ),
    (
        "issue number in a code span",
        "pkg/README.md",
        _readme("`#123` 는 색상이 아니다"),
        _STRICT,
        [],
    ),
    (
        "issue number in an H3 heading",
        "pkg/README.md",
        _readme("## 개요", "### 배경 #123"),
        _STRICT,
        [],
    ),
    (
        "issue number in a link anchor",
        "pkg/README.md",
        _readme("[x](#123-title)", "[y](a.md#45-t)", "a&#123;", "foo#789", "한 자리 #1"),
        _STRICT,
        [],
    ),
    (
        "issue number under 더 읽기 only until the next H2",
        "pkg/README.md",
        _readme("## 더 읽기", "- #123", "### 하위", "- #124", "## 의존성", "- #125"),
        _STRICT,
        ["D14"],
    ),
    (
        "allow marker covers its line",
        "pkg/README.md",
        _readme("<!-- validate-docs: allow D14 -->", "a" * 501, "| a | " + "b" * 300 + " |"),
        _STRICT,
        ["D14"],  # the marker covers the next line only: the table row still fires
    ),
    (
        "allow marker covers a tree block line",
        "pkg/README.md",
        _readme("```", "<!-- validate-docs: allow D14 -->", "├── " + "a" * 60, "```"),
        _STRICT,
        [],
    ),
]
# D15 over a hermetic corpus: README texts that are added to a scratch Repo, then
# (name, citing file, body, codes) through check_readme_section_refs.
SECTION_NAME_READMES = {
    "fx/README.md": (
        "# Pkg\n\n## 흔한 실패와 감별 (colcon · venv)\n\n**Limits and units**\n\n"
        "### Scene overlay (`sim_overlay:=`) — gloss\n\n```\n## Fenced heading\n```\n"
    ),
    "fx/sub/README.md": "# Sub\n\n## Inner topic\n",
    "other/lib/qq/README.md": "# QQ\n\n### Limits\n",
}
SECTION_NAME_CASES: list[tuple[str, str, str, list[str]]] = [
    ("resolves to a heading", "docs/a.md", "see fx/README.md §Scene overlay\n", []),
    ("resolves to the heading's full text", "docs/a.md", "fx/README §Pkg\n", []),
    ("resolves to a bold paragraph", "docs/a.md", "`fx/README.md` §Limits and units\n", []),
    (
        "prefix with trailing prose",
        "docs/a.md",
        "fx/README.md §흔한 실패와 감별 이 소유한다.\n",
        [],
    ),
    ("case and backticks are normalised", "docs/a.md", "fx/README.md §`scene OVERLAY`\n", []),
    ("numeric ref is skipped", "docs/a.md", "fx/README.md §2 table, fx/README.md §7.3\n", []),
    ("unresolved fires", "docs/a.md", "fx/README.md §Nope here\n", ["D15"]),
    ("a longer cited name than the heading", "docs/a.md", "fx/README.md §Pk\n", ["D15"]),
    ("a fenced heading is no name", "docs/a.md", "fx/README.md §Fenced heading\n", ["D15"]),
    ("quoted form equals a name", "docs/a.md", 'fx/README.md "Scene overlay"\n', []),
    ("curly quoted form", "docs/a.md", "fx/README.md “Limits and units”\n", []),
    ("quoted form unresolved", "docs/a.md", 'fx/README "Nope"\n', ["D15"]),
    ("a closing string quote is not a ref", "docs/a.md", 'files = ["README.md", "x"]\n', []),
    ("path may be wrapped in backticks", "docs/a.md", '`fx/README.md` "Nope"\n', ["D15"]),
    (
        "bare README resolves to the nearest ancestor",
        "fx/sub/deep/doc.md",
        'README \u00a7Inner topic and "Nope"\n',
        [],
    ),
    ("bare README, unresolved", "fx/sub/deep/doc.md", "README \u00a7Scene overlay\n", ["D15"]),
    ("bare README in a README refers to itself", "fx/README.md", "README \u00a7Pkg\n", []),
    ("directory word form", "docs/a.md", "fx README \u00a7Scene overlay\n", []),
    ("directory word form in backticks", "docs/a.md", "`fx` README \u00a7Nope\n", ["D15"]),
    ("link form", "docs/a.md", "[L8](../fx/README.md) §Scene overlay\n", []),
    ("link form unresolved", "docs/a.md", "[L8](../fx/README.md) §Nope\n", ["D15"]),
    ("link with an anchor is not form c", "docs/a.md", "[L8](../fx/README.md#x) §Nope\n", []),
    (
        "link label quoting a title uses the link target",
        "fx/sub/doc.md",
        '[루트 README \u0022Scene overlay"](../README.md)\n',
        [],  # without the target this would be fx/sub/README.md, which lacks it
    ),
    (
        "link label quoting a missing title",
        "fx/sub/doc.md",
        '[루트 README \u0022Nope"](../README.md)\n',
        ["D15"],
    ),
    ("unique suffix match", "docs/a.md", 'qq/README "Limits"\n', []),
    ("unique suffix match, unresolved", "docs/a.md", 'qq/README "Nope"\n', ["D15"]),
    ("unresolvable path is skipped", "docs/a.md", "nowhere/README.md §x\n", []),
    ("table cell is cut at the pipe", "docs/a.md", "| fx/README.md §Scene overlay | Nope |\n", []),
    (
        "two refs on a line",
        "docs/a.md",
        "fx/README §Limits and units; fx/README.md §Nope\n",
        ["D15"],
    ),
    (
        "allow marker",
        "docs/a.md",
        "<!-- validate-docs: allow D15 -->\nfx/README.md §Nope\n",
        [],
    ),
    ("fenced block is skipped", "docs/a.md", "```\nfx/README.md §Nope\n```\n", []),
]
SECTION_NAME_SOURCE_CASES: list[tuple[str, str, str, list[str]]] = [
    ("source comment unresolved", "fx/src/x.cpp", "// see README \u00a7Nope\n", ["D15"]),
    ("source comment resolved", "fx/src/x.cpp", "// see README \u00a7Pkg\n", []),
    ("source: fence markers mean nothing", "fx/x.py", "```\n# fx/README.md §Nope\n", ["D15"]),
]
# (name, readme_budget.yaml text, base text, want codes) through check_budget_raises
# with the D14 arguments; and the parse of the `strict` flag.
README_RAISE_CASES: list[tuple[str, str, str, list[str]]] = [
    ("raised without a note", "pkg/README.md: 11\n", "pkg/README.md: 10\n", ["D14"]),
    (
        "raised with its note, strict flag in between",
        "pkg/README.md: 11 strict  # raised from 10: new section\n",
        "pkg/README.md: 10\n",
        [],
    ),
    ("marking strict is not a raise", "pkg/README.md: 10 strict\n", "pkg/README.md: 10\n", []),
    ("lowered", "pkg/README.md: 9 strict\n", "pkg/README.md: 10 strict\n", []),
]
README_STRICT_CASES: list[tuple[str, set[str]]] = [
    (
        "a.md: 1\nb/README.md: 2 strict\nc/README.md: 3 strict  # x\n",
        {"b/README.md", "c/README.md"},
    ),
    ("a/README.md: 1  # strict\n# b/README.md: 2 strict\n", set()),
]


def self_test() -> int:
    failures: list[str] = []

    # One Repo for the whole run.  The document fixtures state their own
    # answers, so the live budget file must not be able to add a finding to (or
    # hide one from) any of them: the budgets are pinned empty here and swapped
    # for a known one where the wiring is tested.  Nothing below reads the live
    # budget file -- whether it matches the tracked documents is a fact about
    # the repository, which the corpus scan reports (check_budget_coverage),
    # not about this checker.  Same for the README budgets (D14).
    repo_for_docs = Repo(repo_root())
    repo_for_docs._budgets = {}
    repo_for_docs._readme = ({}, set())
    for name, rel, body, want_codes in DOC_FIXTURES:
        got_codes = sorted(f.code for f in check_markdown(repo_for_docs, rel, body))
        if got_codes != sorted(want_codes):
            failures.append(f"doc fixture {name!r}: codes={got_codes}, want {sorted(want_codes)}")

    for name, rel, body, want_codes in SOURCE_REF_FIXTURES:
        got_codes = sorted(
            f.code for f in check_section_resolution(repo_for_docs, rel, body, markdown=False)
        )
        if got_codes != sorted(want_codes):
            failures.append(
                f"source fixture {name!r}: codes={got_codes}, want {sorted(want_codes)}"
            )
    # The scope itself: a type or name dropped from the D13 source set leaves
    # every fixture above green while the corpus scan goes blind to it again.
    corpus = set(repo_for_docs.corpus())
    for rel in SOURCE_REF_CORPUS_MEMBERS:
        if repo_for_docs.exists(rel) and rel not in corpus:
            failures.append(f"D13 source scope: {rel} is tracked but not scanned")

    for rel, body, actual, want in COUNT_CASES:
        got_codes = sorted(
            f.code for f in check_package_count(rel, body, actual, suppressions(body))
        )
        if got_codes != sorted(want):
            failures.append(
                f"count case {rel} {body!r} actual={actual}: codes={got_codes}, want {sorted(want)}"
            )

    for name, rel, body, budget_text, want in BUDGET_CASES:
        got_codes = sorted(
            f.code for f in check_size_budget(rel, body, parse_budgets(budget_text))
        )
        if got_codes != sorted(want):
            failures.append(f"budget case {name!r}: codes={got_codes}, want {sorted(want)}")
    for name, sizes, budget_text, existing, want_pairs in BUDGET_COVERAGE_CASES:
        got_pairs = sorted(
            (f.path, f.code)
            for f in check_budget_coverage(sizes, parse_budgets(budget_text), existing)
        )
        if got_pairs != sorted(want_pairs):
            failures.append(f"budget coverage {name!r}: {got_pairs}, want {sorted(want_pairs)}")
    for rel, want in BUDGET_SCOPE_CASES:
        if is_budgeted(rel) != want:
            failures.append(f"is_budgeted({rel!r}) = {not want}, want {want}")
    # The wiring: with the budget reachable only through check_markdown, severing
    # that one call leaves every case above green and the corpus ungated.
    repo_for_docs._budgets = {"agent_docs/f.md": (10, 1)}
    wired_codes = sorted(
        f.code for f in check_markdown(repo_for_docs, "agent_docs/f.md", "a" * 11)
    )
    repo_for_docs._budgets = {}
    if wired_codes != ["D12"]:
        failures.append(f"check_markdown does not apply the size budget: codes={wired_codes}")
    for name, budget_text, base_text, want in BUDGET_RAISE_CASES:
        got_codes = sorted(f.code for f in check_budget_raises(budget_text, base_text))
        if got_codes != sorted(want):
            failures.append(f"budget raise {name!r}: codes={got_codes}, want {sorted(want)}")

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
    repo = repo_for_docs
    if not repo.exists("rtc_base/types/types.hpp"):
        failures.append("suffix match failed for include shorthand rtc_base/types/types.hpp")
    if repo.exists("integrated_bringup/support/owned_topics.cpp"):
        failures.append(
            "suffix match wrongly accepted integrated_bringup/support/owned_topics.cpp"
        )
    if not repo.exists("integrated_bringup/src/support/owned_topics.cpp"):
        failures.append("real path integrated_bringup/src/support/owned_topics.cpp not found")

    # D14: README shape and size.  The cache is pinned per case from the case's own
    # budget text, so the live readme_budget.yaml cannot move any of these answers.
    for name, rel, body, budget_text, want in README_FIXTURES:
        repo_for_docs._readme = (parse_budgets(budget_text), parse_strict(budget_text))
        got_codes = sorted(f.code for f in check_readme(repo_for_docs, rel, body))
        if got_codes != sorted(want):
            failures.append(f"README fixture {name!r}: codes={got_codes}, want {sorted(want)}")
    # The wiring: severing the one check_readme call in check_markdown leaves the
    # cases above green and every README ungated.
    repo_for_docs._readme = ({"pkg/README.md": (10, 1)}, {"pkg/README.md"})
    wired_codes = sorted(
        {f.code for f in check_markdown(repo_for_docs, "pkg/README.md", "a" * 11)}
    )
    if wired_codes != ["D14"]:
        failures.append(f"check_markdown does not apply the README gate: codes={wired_codes}")
    repo_for_docs._readme = ({}, set())
    for name, budget_text, base_text, want in README_RAISE_CASES:
        got_codes = sorted(
            f.code
            for f in check_budget_raises(
                budget_text, base_text, code="D14", budget_file=README_BUDGET_FILE
            )
        )
        if got_codes != sorted(want):
            failures.append(f"README raise {name!r}: codes={got_codes}, want {sorted(want)}")
    for budget_text, want_strict in README_STRICT_CASES:
        if parse_strict(budget_text) != want_strict:
            failures.append(f"parse_strict({budget_text!r}) = {parse_strict(budget_text)}")
    coverage = sorted(
        (f.path, f.code)
        for f in check_budget_coverage(
            {"pkg/README.md": 5},
            parse_budgets("gone/README.md: 9\n"),
            set(),
            code="D14",
            budget_file=README_BUDGET_FILE,
            noun="README",
        )
    )
    if coverage != [("pkg/README.md", "D14"), (README_BUDGET_FILE, "D14")]:
        failures.append(f"README coverage: {coverage}")
    for rel, want in (("README.md", True), ("a/b/README.md", True), ("a/README.mdx", False)):
        if is_readme(rel) != want:
            failures.append(f"is_readme({rel!r}) != {want}")

    # D15: a scratch Repo whose tracked set gains the fixture READMEs.
    repo_names = Repo(repo_root())
    for rel, readme_text in SECTION_NAME_READMES.items():
        repo_names.files.append(rel)
        repo_names.fileset.add(rel)
        repo_names._names_cache[rel] = section_names(readme_text)
    for name, rel, body, want in SECTION_NAME_CASES:
        got_codes = sorted(f.code for f in check_readme_section_refs(repo_names, rel, body))
        if got_codes != sorted(want):
            failures.append(f"README ref {name!r}: codes={got_codes}, want {sorted(want)}")
    for name, rel, body, want in SECTION_NAME_SOURCE_CASES:
        got_codes = sorted(
            f.code for f in check_readme_section_refs(repo_names, rel, body, markdown=False)
        )
        if got_codes != sorted(want):
            failures.append(f"README ref source {name!r}: codes={got_codes}, want {sorted(want)}")
    repo_names._budgets = {}
    repo_names._readme = ({}, set())
    wired_codes = sorted(
        f.code for f in check_markdown(repo_names, "docs/a.md", "fx/README.md \u00a7Nope\n")
    )
    if wired_codes != ["D15"]:
        failures.append(f"check_markdown does not apply D15: codes={wired_codes}")
    failures.extend(scanned_files_failures())

    if failures:
        print("SELF-TEST FAILURES:", file=sys.stderr)
        for f in failures:
            print(f"  - {f}", file=sys.stderr)
        return 1
    total = (
        len(DOC_FIXTURES)
        + len(SOURCE_REF_FIXTURES)
        + len(SOURCE_REF_CORPUS_MEMBERS)
        + len(COUNT_CASES)
        + len(BUDGET_CASES)
        + len(BUDGET_COVERAGE_CASES)
        + len(BUDGET_SCOPE_CASES)
        + len(BUDGET_RAISE_CASES)
        + 1
        + len(GREP_CASES)
        + len(SLUG_CASES)
        + len(BRE_EXEC_CASES)
        + len(PARKED_PATTERN_CASES)
        + len(SETEXT_CASES)
        + 4
        + len(README_FIXTURES)
        + len(README_RAISE_CASES)
        + len(README_STRICT_CASES)
        + 3  # README wiring, coverage, is_readme
        + len(SECTION_NAME_CASES)
        + len(SECTION_NAME_SOURCE_CASES)
        + 1  # D15 wiring
        + 1  # scanned_files
    )
    print(f"self-test: {total} fixtures passed")
    return 0


def scanned_files_failures() -> list[str]:
    """scanned_files on a throwaway repository: tracked, new, ignored, deleted."""
    import tempfile

    with tempfile.TemporaryDirectory() as tmp:
        root = Path(tmp)

        def git(*args: str) -> None:
            subprocess.run(["git", "-C", tmp, *args], check=True, capture_output=True)

        git("init", "-q")
        for rel, body in (
            ("kept.md", "x"),
            ("gone.md", "x"),
            (".gitignore", "ignored.md\n"),
        ):
            (root / rel).write_text(body)
        git("add", ".")
        git("-c", "user.email=t@t", "-c", "user.name=t", "commit", "-qm", "fixture")
        (root / "gone.md").unlink()
        (root / "new.md").write_text("x")
        (root / "ignored.md").write_text("x")
        got = sorted(scanned_files(root))
    want = [".gitignore", "kept.md", "new.md"]
    return [] if got == want else [f"scanned_files: {got}, want {want}"]


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
    ap.add_argument(
        "--section-refs",
        action="store_true",
        help="run only D13 (constitution section refs); the Stop hook uses this "
        "over the whole corpus when a constitution's numbered headings change",
    )
    ap.add_argument(
        "--budget-base",
        metavar="REV",
        help="also fail a byte budget that is higher than at REV without a "
        "'# raised from <old>: <why>' note on its line (CI: the pull request's base)",
    )
    args = ap.parse_args(argv)

    if args.self_test:
        return self_test()

    repo = Repo(repo_root())
    base_text = readme_base_text = ""
    if args.budget_base is not None:
        fetched = base_budget_text(repo, args.budget_base)
        if fetched is None:
            print(f"--budget-base: '{args.budget_base}' is not a commit here", file=sys.stderr)
            return 2
        base_text = fetched
        readme_base_text = base_budget_text(repo, args.budget_base, README_BUDGET_FILE) or ""
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
            if not (rel.endswith(".md") or is_section_ref_source(rel)):
                continue
            # A path that is gone was deleted in this change set; that is not an
            # error, and the hook passes deleted docs through routinely.
            if (repo.root / rel).exists():
                targets.append(rel)
    else:
        targets = repo.corpus()

    findings: list[Finding] = []
    for rel in targets:
        findings.extend(
            check_file_section_refs(repo, rel) if args.section_refs else check_file(repo, rel)
        )
    if not args.section_refs and repo.has_budget_file():
        full_scan = args.files is None
        findings.extend(
            check_budget_coverage(
                {
                    rel: (repo.root / rel).stat().st_size
                    for rel in targets
                    if is_budgeted(rel) and (repo.root / rel).is_file()
                },
                repo.budgets(),
                {f for f in repo.files if is_budgeted(f)} if full_scan else None,
            )
        )

    if not args.section_refs and repo.has_readme_budget_file():
        full_scan = args.files is None
        findings.extend(
            check_budget_coverage(
                {
                    rel: (repo.root / rel).stat().st_size
                    for rel in targets
                    if is_readme(rel) and (repo.root / rel).is_file()
                },
                repo.readme_budgets()[0],
                {f for f in repo.files if is_readme(f)} if full_scan else None,
                code="D14",
                budget_file=README_BUDGET_FILE,
                noun="README",
            )
        )

    for budget_file, base, code in (
        (SIZE_BUDGET_FILE, base_text, "D12"),
        (README_BUDGET_FILE, readme_base_text, "D14"),
    ):
        if base:
            try:
                current = (repo.root / budget_file).read_text(encoding="utf-8")
            except OSError:
                current = ""
            findings.extend(check_budget_raises(current, base, code=code, budget_file=budget_file))

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
