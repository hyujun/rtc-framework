"""Resolve --save-dir default to <session>/plots/.

Figures belong next to the data they were drawn from, so the CSV being plotted
decides the session: we walk *up from the CSV path* to the enclosing
``YYMMDD_HHMM`` directory. Only when the CSV lives outside any session tree do
we fall back to the ambient session (``RTC_SESSION_DIR``, then the most recent
``logging_data/<YYMMDD_HHMM>/``).

Deriving the directory from the environment alone (the earlier behaviour) sent
every figure to the *newest* session, so replotting an older session's CSV
scattered its PNGs into an unrelated session — which ``cleanup_old_sessions()``
then deleted on the 10th subsequent launch.
"""

import os

from rtc_tools.utils.session_dir import (
    get_session_dir,
    is_session_dir_name,
    resolve_logging_root,
)


def find_enclosing_session(path):
    """Return the ``YYMMDD_HHMM`` session directory containing ``path``, or None.

    Walks upward from the file's own directory, so both ``<session>/foo.csv``
    and ``<session>/controllers/<key>/foo.csv`` resolve to ``<session>``.
    """
    cur = os.path.dirname(os.path.abspath(path))
    while True:
        if is_session_dir_name(os.path.basename(cur)):
            return cur
        parent = os.path.dirname(cur)
        if parent == cur:
            return None
        cur = parent


def _latest_session(logging_root):
    """Return the newest ``YYMMDD_HHMM`` dir under ``logging_root``, or None."""
    if not os.path.isdir(logging_root):
        return None
    candidates = sorted(
        d
        for d in os.listdir(logging_root)
        if is_session_dir_name(d) and os.path.isdir(os.path.join(logging_root, d))
    )
    return os.path.join(logging_root, candidates[-1]) if candidates else None


def resolve_default_save_dir(csv_path=None):
    """Return path to <session>/plots/ for the plotted CSV, creating it.

    Resolution order:
      1. session directory enclosing ``csv_path`` (figures land next to the CSV)
      2. ``$RTC_SESSION_DIR`` (live launch session)
      3. most recent ``<logging_root>/YYMMDD_HHMM/``

    Returns None if no session can be located.
    """
    session = find_enclosing_session(csv_path) if csv_path else None
    if not session:
        session = get_session_dir()
    if not session:
        session = _latest_session(resolve_logging_root())

    if not session:
        return None

    save_dir = os.path.join(session, "plots")
    os.makedirs(save_dir, exist_ok=True)
    return save_dir
