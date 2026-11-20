"""Stopping an rclpy spin loop on SIGINT / SIGTERM without losing its work.

rclpy's own signal handlers shut the context down from inside the signal; a
``spin_once`` caught mid-wait or mid-conversion then raises
(``ExternalShutdownException``, a pybind ``RuntimeError``) instead of
``KeyboardInterrupt``, and whatever the tool meant to write on an interrupt is
lost. The tools here init rclpy with ``SignalHandlerOptions.NO`` and let a
flag the loop polls take the signals instead:

    stop = StopFlag()
    with stop_handlers(stop):
        rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
        ...
        spin_until_stopped(lambda: rclpy.spin_once(node, timeout_sec=0.1),
                           rclpy.ok, node.done, stop)
    # the previous handlers are back: a second signal ends the process as usual

Pure Python (no rclpy import): the loop is testable without ROS.
"""

from __future__ import annotations

import signal
from collections.abc import Callable, Iterator
from contextlib import contextmanager


class StopFlag:
    """Set by the signal handlers (it is one), read by the spin loop.

    ``requested`` is any stop. A loop that tells a first Ctrl-C from a second
    one, or Ctrl-C from the end of the terminal, reads ``interrupts`` (SIGINT
    count; a call without a signal counts as one) and ``terminated`` (any
    other signal routed here: SIGTERM, SIGHUP).
    """

    def __init__(self) -> None:
        self.requested = False
        self.interrupts = 0
        self.terminated = False

    def __call__(self, signum: object = None, *_: object) -> None:
        self.requested = True
        if signum is None or signum == signal.SIGINT:
            self.interrupts += 1
        else:
            self.terminated = True


@contextmanager
def stop_handlers(
    flag: StopFlag, signals: tuple[int, ...] = (signal.SIGINT, signal.SIGTERM)
) -> Iterator[StopFlag]:
    """Route `signals` (SIGINT and SIGTERM by default) to `flag` inside the block; the previous handlers return after it."""
    previous = {s: signal.signal(s, flag) for s in signals}
    try:
        yield flag
    finally:
        for s, handler in previous.items():
            signal.signal(s, handler)


def spin_until_stopped(
    spin_once: Callable[[], None],
    ok: Callable[[], bool],
    done: Callable[[], bool],
    stop: StopFlag,
) -> None:
    """Spin while the context is up, the work is not done and `stop` was not requested."""
    while ok() and not stop.requested and not done():
        spin_once()
