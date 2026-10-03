"""
controller_config.py — 컨트롤러 config 한 파일과 그 ``include:`` 조각의 합성

C++ ``rtc_controller_manager/controller_config_loader.hpp`` 의
``LoadControllerConfig`` 와 같은 규칙을 Python 으로 노출합니다. 컨트롤러 config 를
경로로 읽는 도구와 테스트는 ``yaml.safe_load`` 대신 이 모듈을 써야 합니다 — 주
파일만 읽으면 조각의 키가 조용히 빠진 트리를 봅니다.

규칙 (C++ 쪽 헤더가 SSoT, 한쪽을 바꾸면 함께 바꿉니다)::

    include:                  # 주 파일의 top-level, <config_key>: 의 형제
      - catching/part.yaml    # 주 파일의 디렉토리 기준 상대경로 (절대경로 · `..` 거부),
                              #   하위 폴더에 둔다 (`part.yaml` 처럼 같은 폴더는 거부)
    <config_key>:
      ...

- ``include`` 가 없는 파일은 읽은 그대로 돌려줍니다.
- 조각의 top-level 키는 ``<config_key>`` 하나입니다. 조각은 다시 include 하지 못합니다.
- map 은 재귀로 합칩니다. scalar · sequence · null 은 leaf 이고, 같은 leaf 를 두 파일이
  적거나 한쪽이 map 이고 다른 쪽이 leaf 면 에러입니다.
- 키 순서는 주 파일 먼저, 그 뒤 include 순서입니다.
- 각 파일은 YAML 문서 하나이고, 한 map 에 같은 키를 두 번 적으면 에러입니다
  (yaml-cpp 는 첫 값을, PyYAML 은 마지막 값을 읽어 두 로더가 갈립니다).
"""

from __future__ import annotations

from collections.abc import Mapping
from functools import cache
from pathlib import Path, PurePosixPath
from typing import NoReturn

import yaml

INCLUDE_KEY = "include"

# YAML 이 null 로 읽는 scalar. C++ 쪽 `IsNullText` 와 같은 집합입니다.
_NULL_TEXTS = frozenset({"", "~", "null", "Null", "NULL"})


class ControllerConfigIncludeError(ValueError):
    """있기는 하지만 합칠 수 없는 config — 없는 조각, 잘못된 include, 같은 키의 중복."""


def _fail(message: str) -> NoReturn:
    raise ControllerConfigIncludeError(f"controller config include: {message}")


@cache
def _recording(loader):
    """``loader`` 의 하위 클래스 — 한 map 에 두 번 적힌 키를 ``duplicate_keys`` 에 모읍니다."""

    class _DuplicateKeyRecorder(loader):
        def __init__(self, stream):
            super().__init__(stream)
            self.duplicate_keys: list[str] = []

        def construct_mapping(self, node, deep=False):
            seen: set[str] = set()
            for key_node, _ in node.value:
                if not isinstance(key_node, yaml.ScalarNode):
                    continue
                if key_node.value in seen:
                    line = key_node.start_mark.line + 1
                    self.duplicate_keys.append(f"'{key_node.value}' (line {line})")
                seen.add(key_node.value)
            return super().construct_mapping(node, deep=deep)

    return _DuplicateKeyRecorder


def _load_document(text: str, loader) -> tuple[object, list[str]]:
    """문서 하나와, 그 안에서 두 번 적힌 키의 목록."""
    recorder = _recording(loader)(text)
    try:
        return recorder.get_single_data(), recorder.duplicate_keys
    finally:
        recorder.dispose()


def _reject_duplicates(duplicates: list[str], file: object) -> None:
    if duplicates:
        _fail(f"key {duplicates[0]} appears twice in '{file}'")


def _join(parent: str, key: object) -> str:
    return f"{parent}.{key}" if parent else str(key)


def _record_origins(node: object, path: str, file: str, origins: dict[str, str]) -> None:
    if path:
        origins.setdefault(path, file)
    if isinstance(node, Mapping):
        for key, value in node.items():
            _record_origins(value, _join(path, key), file, origins)


def _merge_map(dst: dict, src: Mapping, path: str, src_file: str, origins: dict[str, str]) -> None:
    # yaml-cpp 는 키를 글자로 비교합니다: `1` 과 `"1"` 은 같은 키입니다.
    by_text = {str(key): key for key in dst}
    for key, value in src.items():
        child_path = _join(path, key)
        if str(key) not in by_text:
            dst[key] = value
            by_text[str(key)] = key
            _record_origins(value, child_path, src_file, origins)
            continue
        existing = dst[by_text[str(key)]]
        if isinstance(existing, Mapping) and isinstance(value, Mapping):
            _merge_map(existing, value, child_path, src_file, origins)
            continue
        first_file = origins.get(child_path, "<unknown>")
        if isinstance(existing, Mapping) != isinstance(value, Mapping):
            _fail(
                f"key '{child_path}' is a map in one file and a value in the other "
                f"('{first_file}' and '{src_file}')"
            )
        _fail(f"key '{child_path}' is set in both '{first_file}' and '{src_file}'")


def _resolve_fragment(main_path: Path, entry: object) -> Path:
    if not isinstance(entry, str):
        _fail(f"'{main_path}': every 'include' entry must be a path string")
    if not entry:
        _fail(f"'{main_path}' has an empty 'include' entry")
    relative = PurePosixPath(entry)
    if relative.is_absolute():
        _fail(
            f"'{main_path}' includes '{entry}' — an include path is relative to the "
            "including file's directory, not absolute"
        )
    if ".." in relative.parts:
        _fail(f"'{main_path}' includes '{entry}' — an include path cannot contain '..'")
    if len(relative.parts) < 2:
        _fail(
            f"'{main_path}' includes '{entry}' — a fragment lives in a subdirectory of the "
            "including file's directory"
        )
    return main_path.parent / relative


def _load_fragment(fragment_path: Path, main_path: Path, config_key: str, loader) -> Mapping:
    try:
        text = fragment_path.read_text()
    except OSError:
        _fail(f"'{main_path}' includes '{fragment_path}', which cannot be opened")
    try:
        doc, duplicates = _load_document(text, loader)
    except yaml.YAMLError as exc:
        _fail(f"fragment '{fragment_path}' (included by '{main_path}') does not parse: {exc}")
    if not isinstance(doc, Mapping):
        _fail(
            f"fragment '{fragment_path}' must be a map whose only top-level key is '{config_key}'"
        )
    _reject_duplicates(duplicates, fragment_path)
    for key in doc:
        if key == INCLUDE_KEY:
            _fail(f"fragment '{fragment_path}' has its own 'include' — includes do not nest")
        if key != config_key:
            _fail(
                f"fragment '{fragment_path}' has the top-level key '{key}' — a fragment's "
                f"only top-level key is '{config_key}'"
            )
    body = doc.get(config_key)
    if not isinstance(body, Mapping):
        _fail(f"fragment '{fragment_path}' has no map under '{config_key}'")
    return body


def load_controller_config(
    path: Path | str, *, config_key: str | None = None, loader=yaml.SafeLoader
) -> dict:
    """``path`` 의 문서를 ``include:`` 조각과 합쳐 ``{<config_key>: 트리}`` 로 돌려줍니다.

    ``include`` 가 없는 파일은 읽은 문서를 그대로 돌려줍니다 (빈 파일은 ``{}``).

    ``config_key`` 를 아는 호출자는 넘깁니다 — CM 은 등록된 key 와 파일의 key 가
    다르면 거부하므로, 넘기지 않으면 CM 이 거부할 config 를 받아들일 수 있습니다.
    넘기지 않으면 ``include`` 옆의 유일한 top-level 키를 key 로 봅니다.

    ``loader`` 는 scalar 를 문자열 그대로 비교할 때 ``yaml.BaseLoader`` 로 바꿉니다
    (:func:`controller_config_leaf_lines`).

    주 파일이 없으면 ``OSError`` 가 그대로 올라갑니다. 그 밖의 합성 실패는
    :class:`ControllerConfigIncludeError` 입니다.
    """
    main_path = Path(path)
    doc, duplicates = _load_document(main_path.read_text(), loader)
    doc = doc or {}
    if not isinstance(doc, Mapping) or INCLUDE_KEY not in doc:
        return doc

    _reject_duplicates(duplicates, main_path)
    includes = doc[INCLUDE_KEY]
    if not isinstance(includes, list):
        _fail(f"'{main_path}': the top-level 'include' must be a list of fragment paths")
    keys = [key for key in doc if key != INCLUDE_KEY]
    if config_key is None:
        if len(keys) != 1:
            _fail(
                f"'{main_path}' has an 'include' list, so it has exactly one other top-level "
                f"key (the controller's config key) — found {keys}"
            )
        config_key = keys[0]
    for key in keys:
        if key != config_key:
            _fail(
                f"'{main_path}' has an 'include' list, so its only other top-level key is "
                f"'{config_key}' — found '{key}'"
            )
    composed = doc.get(config_key)
    if not isinstance(composed, Mapping):
        _fail(f"'{main_path}' has an 'include' list but no map under '{config_key}'")

    origins: dict[str, str] = {}
    _record_origins(composed, "", str(main_path), origins)
    for entry in includes:
        fragment_path = _resolve_fragment(main_path, entry)
        body = _load_fragment(fragment_path, main_path, config_key, loader)
        _merge_map(composed, body, "", str(fragment_path), origins)
    return {config_key: composed}


def _render_inline(node: object) -> str:
    if node is None:
        return "null"
    if isinstance(node, Mapping):
        return "{" + ", ".join(f"{k}: {_render_inline(v)}" for k, v in node.items()) + "}"
    if isinstance(node, list):
        return "[" + ", ".join(_render_inline(item) for item in node) + "]"
    text = str(node)
    return "null" if text in _NULL_TEXTS else text


def _append_leaf_lines(node: object, path: str, lines: list[str]) -> None:
    if isinstance(node, Mapping) and node:
        for key, value in node.items():
            _append_leaf_lines(value, _join(path, key), lines)
        return
    lines.append(f"{path}\t{_render_inline(node)}")


def controller_config_leaf_lines(tree: Mapping) -> list[str]:
    """``tree`` 의 leaf 마다 한 줄 — ``<점으로 이은 키 경로>\\t<값>``, 트리 순서.

    C++ ``ControllerConfigLeafLines`` 와 같은 줄을 냅니다 (sequence 는 leaf 하나로
    ``[a, b]``, 그 안의 map 은 ``{k: v}``, null 은 ``null``). scalar 의 글자까지
    C++ 과 같으려면 ``tree`` 를 ``loader=yaml.BaseLoader`` 로 읽어야 합니다 —
    ``SafeLoader`` 는 ``1.0e-4`` 를 float 으로 바꿔 글자가 달라집니다.
    """
    lines: list[str] = []
    for key, value in tree.items():
        _append_leaf_lines(value, str(key), lines)
    return lines
