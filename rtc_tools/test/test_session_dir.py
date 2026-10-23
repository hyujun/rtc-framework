"""Tests for rtc_tools.utils.session_dir module.

C++ ``rtc_base/logging/session_dir.hpp`` 와 Python 헬퍼가 **동일한 로직** 을
공유해야 하므로, 4 단 체인 (env → COLCON_PREFIX_PATH → install/src walk-up →
cwd fallback) 각각을 독립 환경변수 / cwd 조건으로 격리해 검증합니다.
env 가 없을 때 ``get_or_create_session_dir`` 의 폴백 세션 *이름* 은 프로세스 전역입니다 (#930).
"""

from __future__ import annotations

import os
import re
import time

import pytest

from rtc_tools.utils.session_dir import (
    _SESSION_SUBDIRS,
    cleanup_old_sessions,
    create_session_dir,
    get_or_create_session_dir,
    get_session_dir,
    list_session_dirs,
    resolve_logging_root,
)

# ─────────────────────────────────────────────────────────────────────────────
# Fixtures
# ─────────────────────────────────────────────────────────────────────────────

_SESSION_ENV_VARS = ("RTC_SESSION_DIR", "COLCON_PREFIX_PATH")


@pytest.fixture(autouse=True)
def _clean_env(monkeypatch):
    """각 테스트 시작 시 세션 관련 env 변수를 모두 해제."""
    for name in _SESSION_ENV_VARS:
        monkeypatch.delenv(name, raising=False)
    yield


@pytest.fixture
def chdir(monkeypatch):
    """모니터링 주의: pytest monkeypatch 는 cwd 복원을 자동 처리."""

    def _chdir(path):
        monkeypatch.chdir(path)

    return _chdir


@pytest.fixture
def set_tz():
    """TZ 를 바꾸고 ``time.tzset()`` 을 호출하는 함수를 주며, 종료 시 원복한다.

    monkeypatch 의 teardown 순서에 기대지 않고 TZ 를 직접 저장·복원한다.
    """
    saved = os.environ.get("TZ")

    def _set(value):
        os.environ["TZ"] = value
        time.tzset()

    yield _set
    if saved is None:
        os.environ.pop("TZ", None)
    else:
        os.environ["TZ"] = saved
    time.tzset()


def _make_ws(path):
    (path / "install").mkdir(parents=True)
    (path / "src").mkdir()
    return path


# ─────────────────────────────────────────────────────────────────────────────
# resolve_logging_root — 3 단 체인
# ─────────────────────────────────────────────────────────────────────────────


def test_logging_root_uses_colcon_prefix_path_parent(tmp_path, monkeypatch):
    ws = tmp_path / "ws"
    install = ws / "install"
    install.mkdir(parents=True)

    monkeypatch.setenv("COLCON_PREFIX_PATH", str(install))

    assert resolve_logging_root() == str(ws / "logging_data")


def test_logging_root_first_colon_entry_wins(tmp_path, monkeypatch):
    ws1 = tmp_path / "ws1"
    ws2 = tmp_path / "ws2"
    (ws1 / "install").mkdir(parents=True)
    (ws2 / "install").mkdir(parents=True)

    joined = f"{ws1 / 'install'}:{ws2 / 'install'}"
    monkeypatch.setenv("COLCON_PREFIX_PATH", joined)

    assert resolve_logging_root() == str(ws1 / "logging_data")


def test_logging_root_falls_back_when_prefix_missing(tmp_path, monkeypatch, chdir):
    ghost = tmp_path / "ghost" / "install"  # 존재하지 않음
    ws = tmp_path / "real_ws"
    (ws / "install").mkdir(parents=True)
    (ws / "src").mkdir(parents=True)

    monkeypatch.setenv("COLCON_PREFIX_PATH", str(ghost))
    chdir(ws)

    assert resolve_logging_root() == str(ws / "logging_data")


def test_logging_root_walks_up_to_install_src_pair(tmp_path, chdir):
    ws = tmp_path / "walk_ws"
    (ws / "install").mkdir(parents=True)
    deep = ws / "src" / "pkg" / "include" / "subdir"
    deep.mkdir(parents=True)

    chdir(deep)

    assert resolve_logging_root() == str(ws / "logging_data")


def test_logging_root_final_fallback_is_cwd(tmp_path, chdir):
    bare = tmp_path / "bare"
    bare.mkdir()
    chdir(bare)

    assert resolve_logging_root() == str(bare / "logging_data")


# ─────────────────────────────────────────────────────────────────────────────
# create_session_dir — 서브디렉토리 생성
# ─────────────────────────────────────────────────────────────────────────────

_EXPECTED_SUBDIRS = ("timing", "monitor", "device", "sim", "plots", "motions", "tracing")


def test_create_session_dir_makes_all_subdirs(tmp_path):
    session = create_session_dir(str(tmp_path / "logging_data"))

    # 이름은 YYMMDD_HHMM 패턴
    assert os.path.basename(session).count("_") == 1
    name = os.path.basename(session)
    assert len(name) == 11 and name[6] == "_"

    for sub in _EXPECTED_SUBDIRS:
        assert os.path.isdir(os.path.join(session, sub))

    # Legacy singular controller/ subdir was dropped (mirror of C++ session_dir.hpp).
    assert not os.path.isdir(os.path.join(session, "controller"))


def test_create_session_dir_uses_resolve_when_root_missing(tmp_path, chdir):
    chdir(tmp_path)
    session = create_session_dir()
    assert session.startswith(str(tmp_path / "logging_data"))


# ─────────────────────────────────────────────────────────────────────────────
# get_session_dir / get_or_create_session_dir — env 우선
# ─────────────────────────────────────────────────────────────────────────────


def test_get_session_dir_reads_rtc_session_dir(tmp_path, monkeypatch):
    explicit = tmp_path / "explicit"
    monkeypatch.setenv("RTC_SESSION_DIR", str(explicit))
    assert get_session_dir() == str(explicit)


def test_get_session_dir_returns_none_when_unset():
    assert get_session_dir() is None


def test_get_or_create_uses_env_if_set(tmp_path, monkeypatch):
    explicit = tmp_path / "from_env"
    monkeypatch.setenv("RTC_SESSION_DIR", str(explicit))

    session = get_or_create_session_dir()
    assert session == str(explicit)
    for sub in _EXPECTED_SUBDIRS:
        assert os.path.isdir(os.path.join(session, sub))


def test_get_or_create_generates_new_when_unset(tmp_path, chdir):
    chdir(tmp_path)
    session = get_or_create_session_dir()
    assert session.startswith(str(tmp_path / "logging_data"))
    for sub in _EXPECTED_SUBDIRS:
        assert os.path.isdir(os.path.join(session, sub))


# POSIX TZ 의 부호는 반대라 ``-00:01`` 이 UTC+1분 (로컬 시계가 1분 앞섬) 이다.
_TZ_BASE = "AAA0"
_TZ_PLUS_ONE_MIN = "AAA-00:01"


def test_get_or_create_keeps_one_name_per_process(tmp_path, chdir, set_tz):
    ws = _make_ws(tmp_path / "ws")
    chdir(ws)
    set_tz(_TZ_BASE)
    first = get_or_create_session_dir()
    set_tz(_TZ_PLUS_ONE_MIN)
    second = get_or_create_session_dir()

    assert second == first
    assert len(list((ws / "logging_data").iterdir())) == 1


def test_get_or_create_env_overrides_cached_name(tmp_path, chdir, set_tz, monkeypatch):
    ws = _make_ws(tmp_path / "ws")
    chdir(ws)
    set_tz(_TZ_BASE)
    first = get_or_create_session_dir()
    set_tz(_TZ_PLUS_ONE_MIN)

    explicit = tmp_path / "from_env"
    monkeypatch.setenv("RTC_SESSION_DIR", str(explicit))
    assert get_or_create_session_dir() == str(explicit)
    assert len(list((ws / "logging_data").iterdir())) == 1

    monkeypatch.delenv("RTC_SESSION_DIR")
    assert get_or_create_session_dir() == first


def test_get_or_create_reresolves_root_keeps_name(tmp_path, chdir, set_tz):
    ws1 = _make_ws(tmp_path / "ws1")
    ws2 = _make_ws(tmp_path / "ws2")
    chdir(ws1)
    set_tz(_TZ_BASE)
    first = get_or_create_session_dir()
    set_tz(_TZ_PLUS_ONE_MIN)
    chdir(ws2)

    assert get_or_create_session_dir() == str(ws2 / "logging_data" / os.path.basename(first))


# ─────────────────────────────────────────────────────────────────────────────
# cleanup_old_sessions
# ─────────────────────────────────────────────────────────────────────────────


def test_cleanup_old_sessions_keeps_only_max(tmp_path):
    root = tmp_path / "logging_data"
    root.mkdir()
    for name in ("260101_0900", "260102_1000", "260103_1100", "260104_1200", "260105_1300"):
        (root / name).mkdir()
    # 비정상 이름 — 정리 대상 아님
    (root / "not_a_session").mkdir()

    cleanup_old_sessions(str(root), 2)

    remaining = sorted(p.name for p in root.iterdir())
    assert remaining == ["260104_1200", "260105_1300", "not_a_session"]


def test_list_session_dirs_is_sessions_only_oldest_first(tmp_path):
    root = tmp_path / "logging_data"
    root.mkdir()
    for name in ("260103_1100", "260101_0900", "260102_1000", "not_a_session", "stats"):
        (root / name).mkdir()
    # 이름은 세션인데 디렉토리가 아닌 것 — 세션이 아니다
    (root / "260109_0000").write_text("")

    assert list_session_dirs(str(root)) == ["260101_0900", "260102_1000", "260103_1100"]


def test_list_session_dirs_is_empty_without_a_root(tmp_path):
    assert list_session_dirs(str(tmp_path / "does_not_exist")) == []
    not_a_dir = tmp_path / "file"
    not_a_dir.write_text("")
    assert list_session_dirs(str(not_a_dir)) == []


def test_cleanup_old_sessions_no_op_on_missing_root(tmp_path):
    cleanup_old_sessions(str(tmp_path / "does_not_exist"), 2)  # no throw


def test_cleanup_old_sessions_no_op_when_under_limit(tmp_path):
    root = tmp_path / "logging_data"
    root.mkdir()
    (root / "260101_0900").mkdir()
    cleanup_old_sessions(str(root), 5)
    assert (root / "260101_0900").exists()


# ─────────────────────────────────────────────────────────────────────────────
# C++ ↔ Python mirror parity (PROC-5)
# ─────────────────────────────────────────────────────────────────────────────


def _find_cpp_session_dir_header() -> str | None:
    """Locate rtc_base/logging/session_dir.hpp by walking up from this test.

    Returns None when the C++ source is unavailable (e.g. installed-only env),
    so the parity check skips instead of failing.
    """
    rel = os.path.join("rtc_base", "include", "rtc_base", "logging", "session_dir.hpp")
    cur = os.path.dirname(os.path.realpath(__file__))
    while True:
        candidate = os.path.join(cur, rel)
        if os.path.isfile(candidate):
            return candidate
        parent = os.path.dirname(cur)
        if parent == cur:
            return None
        cur = parent


def _parse_cpp_subdirs(header_path: str) -> tuple[str, ...]:
    """Extract the kSubdirs[] string-literal list from the C++ header."""
    with open(header_path, encoding="utf-8") as f:
        text = f.read()
    m = re.search(r"kSubdirs\[\]\s*=\s*\{(.*?)\}", text, re.DOTALL)
    assert m, f"kSubdirs[] initializer not found in {header_path}"
    return tuple(re.findall(r'"([^"]+)"', m.group(1)))


def test_subdir_list_matches_cpp_mirror():
    """PROC-5: _SESSION_SUBDIRS must mirror C++ kSubdirs exactly.

    The launch file (Python) creates the session directory before the C++ node
    starts, so a divergence between the two lists silently resurfaces dropped or
    missing subdirs at runtime.
    """
    header = _find_cpp_session_dir_header()
    if header is None:
        pytest.skip("C++ session_dir.hpp not reachable from this test environment")
    cpp_subdirs = _parse_cpp_subdirs(header)
    assert set(_SESSION_SUBDIRS) == set(cpp_subdirs), (
        f"Python _SESSION_SUBDIRS {sorted(_SESSION_SUBDIRS)} != "
        f"C++ kSubdirs {sorted(cpp_subdirs)} — keep the mirrors in sync (PROC-5)"
    )
