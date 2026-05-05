"""Controller catalog — runtime enumeration via /rtc_cm/list_controllers.

Phase 2 of the demo_controller_gui rework. Replaces the hard-coded
``CONTROLLER_TYPES`` dict with an async lookup against the
``rtc_controller_manager``'s ``list_controllers`` service. The CM is
already advertising this service (see
``rtc_controller_manager/src/rt_controller_node_services.cpp``).

The catalog is pumped by:

  1. an async service call on startup (kick-off in ``ControllerCatalog.start``),
  2. a periodic Tk-thread timer (``poll_interval_s``, default 5 s) that
     re-fires the call until at least one response has succeeded, and
     after that keeps re-querying so a controller hot-swap is reflected
     in the GUI.

Service responses arrive on the rclpy executor thread. The catalog
hands them back to the caller through ``on_update_callback`` which is
invoked on the **rclpy executor thread** — the GUI side is responsible
for marshalling onto the Tk thread (typically via ``root.after(0, …)``).
"""

from __future__ import annotations

import contextlib
from collections.abc import Callable
from dataclasses import dataclass

from rclpy.node import Node

from rtc_msgs.srv import ListControllers


def prettify_config_key(key: str) -> str:
    """Turn ``demo_joint_controller`` into ``Demo Joint Controller``.

    Used as the default display label when no controller-specific
    override is provided. Snake-case aware; preserves any non-ASCII
    characters verbatim (``str.title`` already handles this on Python 3).
    """
    return key.replace("_", " ").title()


@dataclass(frozen=True)
class ControllerEntry:
    """A single row of /rtc_cm/list_controllers, GUI-side enriched."""

    config_key: str  # e.g. 'demo_joint_controller'
    display_label: str  # e.g. 'Demo Joint Controller'
    ctrl_type: str  # registry plugin name
    state: str  # 'unconfigured' | 'inactive' | 'active' | 'finalized'
    is_active: bool
    claimed_groups: tuple[str, ...]
    has_gain_schema: bool  # True iff config_key has a GAIN_DEFS entry


class ControllerCatalog:
    """Lazy enumerator for controllers loaded by the active CM.

    Parameters
    ----------
    node:
        The owning rclpy ``Node`` — used to create the service client +
        the Tk-thread timer's poll callback dispatcher.
    schema_keys:
        Iterable of config keys for which the GUI has a gain panel
        defined (i.e. the keys of ``GAIN_DEFS``). Entries with these
        keys are exposed via ``schema_entries()``; everything else is
        kept around but flagged ``has_gain_schema=False``.
    label_overrides:
        Optional ``{config_key: display_label}`` map, applied before
        falling back to ``prettify_config_key``. Empty by default; the
        Phase 2 design relies on prettify.
    on_update_callback:
        Optional callable fired on every successful service response.
        Receives ``self`` so the callback can read ``schema_entries`` /
        ``is_offline`` directly. Fires on the **rclpy executor thread**.
    poll_interval_s:
        Wall-clock interval between service-call retries. Defaults to
        5 s, matching the sprint plan's "fallback after 5 s" requirement.
    service_wait_timeout_s:
        Per-attempt wait for the service to become reachable. Short by
        design — the catalog is allowed to fail and retry next tick.
    """

    def __init__(
        self,
        node: Node,
        schema_keys: tuple[str, ...] | list[str],
        *,
        label_overrides: dict[str, str] | None = None,
        on_update_callback: Callable[[ControllerCatalog], None] | None = None,
        poll_interval_s: float = 5.0,
        service_wait_timeout_s: float = 0.5,
    ) -> None:
        self._node = node
        self._schema_keys = frozenset(schema_keys)
        self._label_overrides = dict(label_overrides or {})
        self._on_update = on_update_callback
        self._poll_interval_s = float(poll_interval_s)
        self._wait_timeout_s = float(service_wait_timeout_s)

        self._client = node.create_client(ListControllers, "/rtc_cm/list_controllers")
        self._timer = None  # populated in start()
        self._entries: tuple[ControllerEntry, ...] = ()
        self._ever_succeeded: bool = False
        self._inflight: bool = False  # guard against overlapping calls

    # ----------------------------------------------------------------
    # lifecycle

    def start(self) -> None:
        """Kick off the first call + arm the periodic poll timer."""
        # Fire one call right away so the GUI populates as fast as possible
        # if the CM is already up; the timer keeps retrying after 5 s.
        self._maybe_call()
        if self._timer is None:
            self._timer = self._node.create_timer(self._poll_interval_s, self._maybe_call)

    def stop(self) -> None:
        """Cancel the timer + drop the service client. Idempotent."""
        if self._timer is not None:
            with contextlib.suppress(Exception):
                self._node.destroy_timer(self._timer)
            self._timer = None
        if self._client is not None:
            with contextlib.suppress(Exception):
                self._node.destroy_client(self._client)
            self._client = None

    # ----------------------------------------------------------------
    # accessors

    def latest(self) -> tuple[ControllerEntry, ...]:
        """All entries from the last successful response (immutable view)."""
        return self._entries

    def schema_entries(self) -> tuple[ControllerEntry, ...]:
        """Entries whose config_key has a GAIN_DEFS schema in the GUI.

        These are the candidates the controller radio buttons should
        offer — anything else is loaded on the CM but not editable from
        this GUI.
        """
        return tuple(e for e in self._entries if e.has_gain_schema)

    def is_offline(self) -> bool:
        """True until the catalog has received at least one response.

        After a successful response the catalog stays "online" even if
        subsequent retries time out — the GUI keeps showing the last
        known controller list rather than blanking out on transient
        service hiccups.
        """
        return not self._ever_succeeded

    def display_label(self, config_key: str) -> str:
        """Human-friendly label for ``config_key``.

        Resolution order:
          1. live entry from the latest successful response,
          2. ``label_overrides`` mapping passed at construction,
          3. ``prettify_config_key`` fallback.
        """
        for e in self._entries:
            if e.config_key == config_key:
                return e.display_label
        if config_key in self._label_overrides:
            return self._label_overrides[config_key]
        return prettify_config_key(config_key)

    # ----------------------------------------------------------------
    # internals

    def _maybe_call(self) -> None:
        """Send a ListControllers request unless one is already in flight."""
        if self._client is None or self._inflight:
            return
        if not self._client.wait_for_service(timeout_sec=self._wait_timeout_s):
            return  # CM not up yet; the periodic timer will retry
        self._inflight = True
        future = self._client.call_async(ListControllers.Request())
        future.add_done_callback(self._on_response)

    def _on_response(self, future) -> None:
        """Service response handler — runs on the rclpy executor thread."""
        self._inflight = False
        try:
            resp = future.result()
        except Exception as exc:  # noqa: BLE001 — informational logging
            self._node.get_logger().debug(f"list_controllers call failed: {exc}")
            return
        if resp is None:
            return

        entries = []
        for cs in resp.controllers:
            label = self._label_overrides.get(cs.name, prettify_config_key(cs.name))
            entries.append(
                ControllerEntry(
                    config_key=cs.name,
                    display_label=label,
                    ctrl_type=cs.type,
                    state=cs.state,
                    is_active=cs.is_active,
                    claimed_groups=tuple(cs.claimed_groups),
                    has_gain_schema=(cs.name in self._schema_keys),
                )
            )
        self._entries = tuple(entries)
        self._ever_succeeded = True

        if self._on_update is not None:
            try:
                self._on_update(self)
            except Exception as exc:  # noqa: BLE001 — keep the executor alive
                self._node.get_logger().error(f"controller catalog callback raised: {exc}")
