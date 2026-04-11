"""Tests for rtc_tools.utils.session_dir module.

C++ ``rtc_base/logging/session_dir.hpp`` 와 Python 헬퍼가 **동일한 로직** 을
공유해야 하므로, 4 단 체인 (env → COLCON_PREFIX_PATH → install/src walk-up →
cwd fallback) 각각을 독립 환경변수 / cwd 조건으로 격리해 검증합니다.
"""

from __future__ import annotations

import os

import pytest

from rtc_tools.utils.session_dir import (
    cleanup_old_sessions,
    create_session_dir,
    get_or_create_session_dir,
    get_session_dir,
    resolve_logging_root,
)


# ─────────────────────────────────────────────────────────────────────────────
# Fixtures
# ─────────────────────────────────────────────────────────────────────────────

_SESSION_ENV_VARS = ('RTC_SESSION_DIR', 'UR5E_SESSION_DIR', 'COLCON_PREFIX_PATH')


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


# ─────────────────────────────────────────────────────────────────────────────
# resolve_logging_root — 3 단 체인
# ─────────────────────────────────────────────────────────────────────────────

def test_logging_root_uses_colcon_prefix_path_parent(tmp_path, monkeypatch):
    ws = tmp_path / "ws"
    install = ws / "install"
    install.mkdir(parents=True)

    monkeypatch.setenv('COLCON_PREFIX_PATH', str(install))

    assert resolve_logging_root() == str(ws / "logging_data")


def test_logging_root_first_colon_entry_wins(tmp_path, monkeypatch):
    ws1 = tmp_path / "ws1"
    ws2 = tmp_path / "ws2"
    (ws1 / "install").mkdir(parents=True)
    (ws2 / "install").mkdir(parents=True)

    joined = f"{ws1 / 'install'}:{ws2 / 'install'}"
    monkeypatch.setenv('COLCON_PREFIX_PATH', joined)

    assert resolve_logging_root() == str(ws1 / "logging_data")


def test_logging_root_falls_back_when_prefix_missing(tmp_path, monkeypatch, chdir):
    ghost = tmp_path / "ghost" / "install"  # 존재하지 않음
    ws = tmp_path / "real_ws"
    (ws / "install").mkdir(parents=True)
    (ws / "src").mkdir(parents=True)

    monkeypatch.setenv('COLCON_PREFIX_PATH', str(ghost))
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

_EXPECTED_SUBDIRS = ('controller', 'monitor', 'device', 'sim', 'plots', 'motions')


def test_create_session_dir_makes_all_subdirs(tmp_path):
    session = create_session_dir(str(tmp_path / "logging_data"))

    # 이름은 YYMMDD_HHMM 패턴
    assert os.path.basename(session).count('_') == 1
    name = os.path.basename(session)
    assert len(name) == 11 and name[6] == '_'

    for sub in _EXPECTED_SUBDIRS:
        assert os.path.isdir(os.path.join(session, sub))


def test_create_session_dir_uses_resolve_when_root_missing(tmp_path, chdir):
    chdir(tmp_path)
    session = create_session_dir()
    assert session.startswith(str(tmp_path / "logging_data"))


# ─────────────────────────────────────────────────────────────────────────────
# get_session_dir / get_or_create_session_dir — env 우선
# ─────────────────────────────────────────────────────────────────────────────

def test_get_session_dir_reads_rtc_session_dir(tmp_path, monkeypatch):
    explicit = tmp_path / "explicit"
    monkeypatch.setenv('RTC_SESSION_DIR', str(explicit))
    assert get_session_dir() == str(explicit)


def test_get_session_dir_falls_back_to_legacy(tmp_path, monkeypatch):
    legacy = tmp_path / "legacy"
    monkeypatch.setenv('UR5E_SESSION_DIR', str(legacy))
    assert get_session_dir() == str(legacy)


def test_get_session_dir_returns_none_when_unset():
    assert get_session_dir() is None


def test_get_or_create_uses_env_if_set(tmp_path, monkeypatch):
    explicit = tmp_path / "from_env"
    monkeypatch.setenv('RTC_SESSION_DIR', str(explicit))

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


# ─────────────────────────────────────────────────────────────────────────────
# cleanup_old_sessions
# ─────────────────────────────────────────────────────────────────────────────

def test_cleanup_old_sessions_keeps_only_max(tmp_path):
    root = tmp_path / "logging_data"
    root.mkdir()
    for name in ('260101_0900', '260102_1000', '260103_1100',
                 '260104_1200', '260105_1300'):
        (root / name).mkdir()
    # 비정상 이름 — 정리 대상 아님
    (root / "not_a_session").mkdir()

    cleanup_old_sessions(str(root), 2)

    remaining = sorted(p.name for p in root.iterdir())
    assert remaining == ['260104_1200', '260105_1300', 'not_a_session']


def test_cleanup_old_sessions_no_op_on_missing_root(tmp_path):
    cleanup_old_sessions(str(tmp_path / "does_not_exist"), 2)  # no throw


def test_cleanup_old_sessions_no_op_when_under_limit(tmp_path):
    root = tmp_path / "logging_data"
    root.mkdir()
    (root / "260101_0900").mkdir()
    cleanup_old_sessions(str(root), 5)
    assert (root / "260101_0900").exists()
