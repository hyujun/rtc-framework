"""ros_stop: a signal sets the flag the spin loop polls (no context shutdown, no
KeyboardInterrupt), the handlers come back after the block, and the loop ends on the flag or
on done. Pure Python: no rclpy."""

from __future__ import annotations

import os
import signal
import time

import pytest

from rtc_tools.analysis import ros_stop

SPIN_CAP = 50  # a loop that ignores its exit condition fails here instead of hanging


class Spinner:
    """Counts spins; `on` maps a spin number to a callable run at that spin."""

    def __init__(self, on: dict | None = None) -> None:
        self.spins = 0
        self.on = on or {}

    def __call__(self) -> None:
        self.spins += 1
        assert self.spins <= SPIN_CAP, "the loop did not stop"
        action = self.on.get(self.spins)
        if action is not None:
            action()


def test_a_stop_request_ends_the_loop_after_the_spin_in_flight():
    flag = ros_stop.StopFlag()
    spinner = Spinner({3: flag})  # the signal lands while spinning
    ros_stop.spin_until_stopped(spinner, lambda: True, lambda: False, flag)
    assert spinner.spins == 3


def test_done_ends_the_loop_without_a_stop():
    spinner = Spinner()
    ros_stop.spin_until_stopped(
        spinner, lambda: True, lambda: spinner.spins >= 2, ros_stop.StopFlag()
    )
    assert spinner.spins == 2


def test_a_context_that_is_down_ends_the_loop():
    spinner = Spinner()
    ros_stop.spin_until_stopped(spinner, lambda: False, lambda: False, ros_stop.StopFlag())
    assert spinner.spins == 0


def _send_to_self(sig: int, flag: ros_stop.StopFlag) -> None:
    """Deliver `sig` to this thread: blocked while sent, delivered on the unblock; then give
    the Python-level handler (run at the next eval-breaker check) up to 100 ms."""
    signal.pthread_sigmask(signal.SIG_BLOCK, [sig])
    os.kill(os.getpid(), sig)
    signal.pthread_sigmask(signal.SIG_UNBLOCK, [sig])
    for _ in range(100):
        if flag.requested:
            return
        time.sleep(0.001)


@pytest.mark.parametrize("sig", [signal.SIGINT, signal.SIGTERM], ids=signal.strsignal)
def test_a_signal_inside_the_block_sets_the_flag_and_raises_nothing(sig):
    flag = ros_stop.StopFlag()
    before = signal.getsignal(sig)
    with ros_stop.stop_handlers(flag):
        assert signal.getsignal(sig) is flag
        try:
            _send_to_self(sig, flag)
        except KeyboardInterrupt:
            pytest.fail(f"{signal.strsignal(sig)} raised KeyboardInterrupt")
        assert flag.requested
    assert signal.getsignal(sig) is before, "the previous handler is back after the block"


def test_the_flag_counts_interrupts_and_marks_other_signals():
    # A loop that ends at a cycle boundary on the first Ctrl-C and at once on
    # the second, or on the terminal going away, reads these two.
    flag = ros_stop.StopFlag()
    flag(signal.SIGINT, None)
    assert (flag.requested, flag.interrupts, flag.terminated) == (True, 1, False)
    flag(signal.SIGINT, None)
    flag(signal.SIGHUP, None)
    assert (flag.interrupts, flag.terminated) == (2, True)
    plain = ros_stop.StopFlag()
    plain()
    assert (plain.requested, plain.interrupts) == (True, 1)


def test_the_routed_signals_can_include_sighup():
    flag = ros_stop.StopFlag()
    before = signal.getsignal(signal.SIGHUP)
    signals = (signal.SIGINT, signal.SIGTERM, signal.SIGHUP)
    with ros_stop.stop_handlers(flag, signals=signals):
        assert signal.getsignal(signal.SIGHUP) is flag
        _send_to_self(signal.SIGHUP, flag)
        assert flag.terminated and flag.interrupts == 0
    assert signal.getsignal(signal.SIGHUP) is before


def test_the_previous_handlers_come_back_even_when_the_block_raises():
    flag = ros_stop.StopFlag()
    before = {s: signal.getsignal(s) for s in (signal.SIGINT, signal.SIGTERM)}
    with pytest.raises(RuntimeError), ros_stop.stop_handlers(flag):
        raise RuntimeError("the tool failed inside the block")
    assert {s: signal.getsignal(s) for s in before} == before
