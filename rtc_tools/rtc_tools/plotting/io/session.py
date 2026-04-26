"""Resolve --save-dir default to <session>/plots/.

Walks `RTC_SESSION_DIR` env var first, then falls back to the most recent
`logging_data/<YYMMDD_HHMM>/` session directory (11-char `YYMMDD_HHMM` pattern).
"""

import os


def resolve_default_save_dir():
    """Return path to <session>/plots/ for the active session, creating it.

    Returns None if no session can be located.
    """
    from rtc_tools.utils.session_dir import get_session_dir, resolve_logging_root

    session = get_session_dir()
    if not session:
        root = resolve_logging_root()
        if os.path.isdir(root):
            candidates = sorted(
                d
                for d in os.listdir(root)
                if os.path.isdir(os.path.join(root, d))
                and len(d) == 11
                and d[6] == "_"
            )
            if candidates:
                session = os.path.join(root, candidates[-1])

    if not session:
        return None

    save_dir = os.path.join(session, "plots")
    os.makedirs(save_dir, exist_ok=True)
    return save_dir
