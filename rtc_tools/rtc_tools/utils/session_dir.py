"""
session_dir.py — 세션 디렉토리 유틸리티

C++ ``rtc_base/logging/session_dir.hpp`` 와 동일한 결정 로직을 Python 으로
노출합니다. launch 파일과 GUI/플롯 툴 모두가 이 모듈을 사용해 같은 경로에
세션을 생성·재사용하도록 하는 것이 목적입니다.

로깅 루트 결정 체인 (``resolve_logging_root``)::

    1. $COLCON_PREFIX_PATH 의 첫 entry 가 쓰기 가능한 디렉토리이면
       그 parent / "logging_data"
    2. cwd 에서 상위로 올라가며 install/ + src/ 쌍을 찾으면 그 dir / "logging_data"
    3. 최종 폴백: $PWD / "logging_data"

세션 디렉토리 결정 체인 (``get_session_dir`` / ``create_session_dir``)::

    1. $RTC_SESSION_DIR → $UR5E_SESSION_DIR (하위 호환)
    2. resolve_logging_root() / "YYMMDD_HHMM"
"""

import os
import re
import shutil
from datetime import datetime
from typing import Optional

_SESSION_SUBDIRS = (
    'controller', 'monitor', 'device', 'sim', 'plots', 'motions',
)
_SESSION_PATTERN = re.compile(r'^\d{6}_\d{4}$')


def resolve_logging_root() -> str:
    """활성 colcon 워크스페이스의 ``logging_data`` 루트 경로를 결정합니다.

    C++ ``rtc::ResolveLoggingRoot()`` 와 동일한 3단 체인.
    """
    # 1) COLCON_PREFIX_PATH 첫 entry 의 parent
    prefix_env = os.environ.get('COLCON_PREFIX_PATH', '')
    if prefix_env:
        first = prefix_env.split(':', 1)[0]
        if first and os.path.isdir(first) and os.access(first, os.W_OK):
            return os.path.join(os.path.dirname(first), 'logging_data')

    # 2) cwd 상위 탐색 (install/ + src/ 쌍)
    cur = os.getcwd()
    while True:
        if (os.path.isdir(os.path.join(cur, 'install')) and
                os.path.isdir(os.path.join(cur, 'src'))):
            return os.path.join(cur, 'logging_data')
        parent = os.path.dirname(cur)
        if parent == cur:
            break
        cur = parent

    # 3) 최종 폴백
    return os.path.join(os.getcwd(), 'logging_data')


def _ensure_session_subdirs(session_dir: str) -> None:
    for sub in _SESSION_SUBDIRS:
        os.makedirs(os.path.join(session_dir, sub), exist_ok=True)


def create_session_dir(logging_root: Optional[str] = None) -> str:
    """현재 타임스탬프로 새 세션 디렉토리를 만들고 표준 서브디렉토리까지 생성.

    Args:
        logging_root: 루트 경로. ``None`` 이면 :func:`resolve_logging_root` 사용.

    Returns:
        생성된 세션 디렉토리 절대 경로.
    """
    root = logging_root if logging_root else resolve_logging_root()
    session = os.path.join(root, datetime.now().strftime('%y%m%d_%H%M'))
    _ensure_session_subdirs(session)
    return session


def cleanup_old_sessions(logging_root: str, max_sessions: int) -> None:
    """YYMMDD_HHMM 패턴 세션 폴더를 ``max_sessions`` 개 이하로 유지."""
    if max_sessions <= 0 or not os.path.isdir(logging_root):
        return
    dirs = sorted(
        d for d in os.listdir(logging_root)
        if os.path.isdir(os.path.join(logging_root, d))
        and _SESSION_PATTERN.match(d)
    )
    while len(dirs) > max_sessions:
        oldest = os.path.join(logging_root, dirs.pop(0))
        shutil.rmtree(oldest, ignore_errors=True)


def get_session_dir() -> Optional[str]:
    """``RTC_SESSION_DIR`` / ``UR5E_SESSION_DIR`` 환경변수에서 세션 경로를 읽어옵니다.

    런치 파일이 자식 노드에게 전파한 세션 경로를 다시 읽을 때 사용합니다.

    Returns:
        세션 디렉토리 경로. 둘 다 비어있으면 ``None``.
    """
    val = os.environ.get('RTC_SESSION_DIR', '')
    if not val:
        val = os.environ.get('UR5E_SESSION_DIR', '')  # backward compat
    return val if val else None


def get_or_create_session_dir() -> str:
    """환경변수에 세션이 설정돼 있으면 그것을, 아니면 새 세션을 만들어 반환.

    CLI 툴(플롯/모션 편집기 등) 이 "런치 중인 세션이 있으면 그걸, 없으면
    현재 ws 에 새로 만들어 거기에 저장" 하는 흐름을 한 줄로 쓸 때 사용.
    """
    existing = get_session_dir()
    if existing:
        _ensure_session_subdirs(existing)
        return existing
    return create_session_dir()


def get_session_subdir(subdir: str) -> Optional[str]:
    """현재 세션 디렉토리 내 특정 서브디렉토리 경로를 반환합니다.

    Args:
        subdir: 서브디렉토리 이름 (e.g. ``'plots'``, ``'device'``, ``'controller'``).

    Returns:
        서브디렉토리 절대 경로. 세션 미설정 시 ``None``.
    """
    session = get_session_dir()
    if session is None:
        return None
    path = os.path.join(session, subdir)
    os.makedirs(path, exist_ok=True)
    return path
