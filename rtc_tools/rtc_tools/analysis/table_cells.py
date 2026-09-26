"""Reading one cell of a per-trial table — shared by the catching analysis modules.

A row is a dict built in memory (``catching_trials``) or parsed back from a CSV
(``catching_pool``), so a cell can be a float, an int, a bool, ``None`` or a
string. These are the only two readings the statistics need.
"""

from __future__ import annotations

import math

import numpy as np


def num(value) -> float:
    """A cell as a float (``None`` / a string / non-numeric → NaN)."""
    if value is None or isinstance(value, str):
        return math.nan
    try:
        return float(value)
    except (TypeError, ValueError):
        return math.nan


def is_true(value) -> bool:
    """A real boolean True (a NaN / ``None`` / missing cell is not a success)."""
    return isinstance(value, bool | np.bool_) and bool(value)
