"""Cached parameter readouts for demo_controller_gui (#852) — Tk- and rclpy-free.

The Grasp tab's mode readout and the Control tab's Virtual TCP panel both show
a value read from the selected controller's parameters, and both read it the
same way. That way is the contract this module holds, so the two panels cannot
drift apart again (#852 was the grasp mode keeping an older copy of it):

- a cache hit paints without a round trip — the 5 s catalog poll and every
  radio click land here;
- a miss paints the panel's unknown value first, so the panel never keeps
  showing the *previous* controller's value while the read is in flight;
- ``force`` re-reads past the cache (after a set, and for an explicit Load)
  and paints nothing until the answer lands — the cached value stays on
  screen, so a poll in between repaints it rather than unknown;
- readiness is checked without blocking (this runs on the Tk thread), and a
  not-ready service leaves the panel as it is for the next poll to retry;
- an answer that does not parse is cached as nothing, and a failed read drops
  the entry, because a forced read follows a change that may have landed.

What the panels do differently — the parameters, how an answer parses, what
unknown looks like, how a value is painted — is passed in.
"""

from __future__ import annotations

from collections.abc import Callable, Collection, Sequence
from dataclasses import dataclass
from typing import Any


@dataclass(frozen=True)
class ReadoutSpec[T]:
    """What one panel reads: the parameters, who declares them, and how."""

    params: tuple[str, ...]  # get_parameters request order; parse reads it in this order
    owners: Collection[str]  # controllers that declare ``params``; others are not queried
    parse: Callable[[Sequence[Any]], T | None]  # ParameterValue list -> value, None if unreadable
    unknown: T  # painted while a non-forced read is in flight, and for a non-owner
    label: str  # names the parameters in log lines


class CachedParamReadout[T]:
    """Reads ``spec.params`` from a controller and paints the result.

    ``cache`` is the caller's dict (controller key -> value) and is shared, not
    copied. ``get_client(ctrl)`` returns an AsyncParameterClient-shaped object;
    ``paint(ctrl, value)`` runs on the Tk thread and drops a value for a
    controller that is no longer selected; ``defer(fn, *args)`` marshals onto
    the Tk thread from the executor thread the answer arrives on.
    """

    def __init__(
        self,
        spec: ReadoutSpec[T],
        cache: dict[str, T],
        *,
        get_client: Callable[[str], Any],
        paint: Callable[[str, T], None],
        defer: Callable[..., None],
        logger: Callable[[], Any],
    ) -> None:
        self.spec = spec
        self.cache = cache
        self._get_client = get_client
        self._paint = paint
        self._defer = defer
        self._logger = logger

    def refresh(self, ctrl: str, *, force: bool = False) -> None:
        """Show ``ctrl``'s value, reading it if not cached or if ``force``. Tk thread."""
        if not ctrl:
            return
        cached = None if force else self.cache.get(ctrl)
        if cached is not None:
            self._paint(ctrl, cached)
            return
        if not force:
            self._paint(ctrl, self.spec.unknown)
        if ctrl not in self.spec.owners:
            # No parameter to read — the unknown paint is already the answer.
            return

        client = self._get_client(ctrl)
        # Non-blocking, NOT a timed wait: any timeout here would freeze the
        # window on every poll for as long as the CM is down.
        if not client.services_are_ready():
            return

        future = client.get_parameters(list(self.spec.params))
        label = self.spec.label

        def _on_done(fut):
            try:
                resp = fut.result()
            except Exception as exc:  # noqa: BLE001 — any failure = unknown
                self._logger().debug(f"{label} query failed for /{ctrl}: {exc}")
                # A forced read follows a change, so the cached value may be the
                # one that changed; stop claiming it.
                self.cache.pop(ctrl, None)
                return
            value = self.spec.parse(list(resp.values))
            if value is None:
                # Not declared (an older controller) or not the declared shape.
                # Indistinguishable from unknown, and cached as neither.
                self._logger().debug(f"/{ctrl} {label} unreadable: {resp.values}")
                return
            self.cache[ctrl] = value
            self._logger().info(f"/{ctrl} {label}={value}")
            self._defer(self._paint, ctrl, value)

        future.add_done_callback(_on_done)
