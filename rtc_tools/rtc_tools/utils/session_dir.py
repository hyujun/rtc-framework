"""
session_dir.py — 세션 디렉토리 유틸리티

UR5E_SESSION_DIR 환경변수에서 현재 세션 디렉토리를 읽어오는 헬퍼.
C++ session_dir.hpp와 동일한 세션 구조를 공유합니다.

사용법:
    from rtc_tools.utils.session_dir import get_session_dir, get_session_subdir

    session = get_session_dir()        # None if not set
    plots_dir = get_session_subdir('plots')  # .../YYMMDD_HHMM/plots or None
"""

import os
from typing import Optional


def get_session_dir() -> Optional[str]:
    """UR5E_SESSION_DIR 환경변수에서 세션 디렉토리 경로를 읽어옵니다.

    Returns:
        세션 디렉토리 경로 문자열, 미설정 시 None
    """
    val = os.environ.get('RTC_SESSION_DIR', '')
    if not val:
        val = os.environ.get('UR5E_SESSION_DIR', '')  # backward compat
    return val if val else None


def get_session_subdir(subdir: str) -> Optional[str]:
    """세션 디렉토리 내 특정 서브디렉토리 경로를 반환합니다.

    Args:
        subdir: 서브디렉토리 이름 (e.g. 'plots', 'hand', 'controller')

    Returns:
        서브디렉토리 절대 경로 문자열, 세션 미설정 시 None
    """
    session = get_session_dir()
    if session is None:
        return None
    path = os.path.join(session, subdir)
    os.makedirs(path, exist_ok=True)
    return path
