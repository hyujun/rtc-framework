"""Smoothing shared by the analysis tools and the plotters.

One kernel, so a figure and a table computed from the same log agree: the
catching_diag command-kinematics panel and catching_trials' ``cmd_*`` columns
both differentiate FK or joint commands recorded every 2 ms, where a second
difference is dominated by tick noise.
"""

from __future__ import annotations

import numpy as np

# Rows the catching tools average a differentiated command over (10 ms at the
# 2 ms tick the catching controller logs at).
COMMAND_SMOOTH_ROWS = 5


def box_smooth(x: np.ndarray, rows: int) -> np.ndarray:
    """Centred moving average over ``rows`` rows along axis 0, same length.

    The window SHRINKS at the two ends instead of being padded: zero padding
    pulls the ends toward zero and edge padding repeats a value that was
    never recorded, and either shows up as a kink in a derivative taken
    afterwards. A NaN spreads to the rows whose window holds it. ``rows`` ≤ 1
    returns ``x`` unchanged.
    """
    x = np.asarray(x, dtype=float)
    if rows <= 1 or len(x) == 0:
        return x
    # `full` and a centred cut, not `same`: with fewer rows than the window
    # `same` returns the WINDOW's length, not the input's.
    lead = (rows - 1) // 2
    kernel = np.ones(rows)

    def centred(v: np.ndarray) -> np.ndarray:
        return np.convolve(v, kernel, mode="full")[lead : lead + len(x)]

    norm = centred(np.ones(len(x)))
    if x.ndim == 1:
        return centred(x) / norm
    return np.stack([centred(x[:, i]) for i in range(x.shape[1])], axis=1) / norm[:, None]
