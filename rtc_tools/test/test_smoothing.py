"""rtc_tools.utils.smoothing — the one box filter the catching tools share."""

from __future__ import annotations

import numpy as np

from rtc_tools.utils.smoothing import COMMAND_SMOOTH_ROWS, box_smooth


def test_a_constant_stays_constant_up_to_both_ends():
    # The window shrinks at the ends: zero padding would pull them toward 0,
    # and a derivative taken afterwards would show a kink there.
    out = box_smooth(np.full(20, 3.0), 5)
    assert out.shape == (20,)
    assert np.allclose(out, 3.0)


def test_a_ramp_keeps_its_slope_in_the_interior_and_its_range_at_the_ends():
    x = np.arange(20.0)
    out = box_smooth(x, 5)
    assert np.allclose(out[2:-2], x[2:-2])  # a centred average of a line is the line
    assert out[0] == np.mean(x[:3]) and out[-1] == np.mean(x[-3:])


def test_it_averages_and_works_per_column():
    x = np.zeros((9, 2))
    x[4, 0] = 5.0
    x[:, 1] = 1.0
    out = box_smooth(x, 5)
    assert out.shape == (9, 2)
    assert np.allclose(out[2:7, 0], 1.0) and out[1, 0] == 0.0 and out[7, 0] == 0.0
    assert np.allclose(out[:, 1], 1.0)


def test_a_nan_spreads_to_the_rows_whose_window_holds_it_and_no_further():
    x = np.ones(11)
    x[5] = np.nan
    out = box_smooth(x, 5)
    assert np.isnan(out[3:8]).all()
    assert np.allclose(out[:3], 1.0) and np.allclose(out[8:], 1.0)


def test_an_input_shorter_than_the_window_keeps_its_length():
    # np.convolve(mode="same") returns the longer of its two inputs: a
    # three-row log through a five-row window would come back five rows long.
    x = np.array([1.0, 2.0, 6.0])
    out = box_smooth(x, 5)
    assert out.shape == (3,)
    assert np.allclose(out, 3.0)  # every window holds all three rows
    two_d = box_smooth(np.column_stack([x, x]), 5)
    assert two_d.shape == (3, 2)
    even = box_smooth(np.arange(6.0), 4)  # an even window keeps the length too
    assert even.shape == (6,)


def test_one_row_or_less_is_the_input():
    x = np.array([1.0, 4.0, 2.0])
    assert np.array_equal(box_smooth(x, 1), x)
    assert np.array_equal(box_smooth(x, 0), x)
    assert box_smooth(np.array([]), 5).shape == (0,)
    assert COMMAND_SMOOTH_ROWS == 5
