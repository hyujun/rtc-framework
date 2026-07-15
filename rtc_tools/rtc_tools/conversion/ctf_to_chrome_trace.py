"""Convert an LTTng CTF trace into a Chrome Trace JSON for Perfetto UI.

Input: a directory containing an LTTng CTF trace (the output of
``ros2 launch ... enable_tracing:=true`` lives at
``<ws>/logging_data/<YYMMDD_HHMM>/tracing/<session_name>/``; manual
``ros2 trace`` CLI captures land under ``~/.ros/tracing/``). The directory
is expected to contain ``metadata`` plus one or more channel binaries —
babeltrace2 walks any sub-tree automatically.

Output: a Chrome Trace JSON file. Drag-drop it onto
https://ui.perfetto.dev to see the swimlane groups, top to bottom:

* **Workspace processes (focus tier)** — one Perfetto process group per
  real vpid that emitted ``rtc:span_*`` (i.e. carries workspace
  ``RTC_TRACE_SCOPE`` instrumentation), with one row per thread. Use this
  to inspect *what each workspace thread was doing* over time.
* **Cpus** — sched_switch data per ``cpu_id``, each slice labelled
  ``process/comm-tid`` (process prefix from the vpid context). Use this
  to verify ApplyThreadConfig pinning and spot migrations or IRQ leaks.
* **External process summaries** — UST-emitting processes *without*
  ``rtc:*`` spans (e.g. a vendor driver node) collapse into one async
  lane per process: callback slices survive, but they no longer occupy
  one row per tid. ``--all-threads`` restores per-thread rows;
  ``--focus-proc <substr,...>`` force-promotes processes into the focus
  tier (needed for captures from a build without RTC_ENABLE_TRACING,
  where no span exists to classify automatically — that case falls back
  to per-thread detail everywhere, with a stderr note).

Threads visible only through ``sched_switch`` (kernel tasks, non-UST
system processes) get no thread row at all — they appear on the Cpu
lanes only. Captures without the vpid context land in a flat
``Threads (by TID)`` fallback group, always at per-thread detail.

Unlike the prior perf-sampling converter, ros2_tracing events carry
*exact* begin/end timestamps — no common-prefix stitching guesswork. The
events handled today:

* ``ros2:callback_start`` / ``ros2:callback_end`` — paired into
  ``B`` / ``E`` slices on the (process_name, tid) lane. The slice is
  labelled with the callback's function symbol, resolved through the
  address→symbol map built from ``ros2:rclcpp_callback_register`` events
  (part of ros2_tracing's default UST set). Falls back to the raw
  address when no registration was captured (e.g. a manual ``ros2 trace``
  started after node init, or a narrowed ``trace_events_ust`` list).
* ``rtc:span_begin`` / ``rtc:span_end`` — rtc_base ``RTC_TRACE_SCOPE``
  spans, paired into ``B`` / ``E`` slices on the (process_name, tid) lane
  and labelled from the begin event's ``name`` field. These expose RT-tick
  internals — control loop → controller ``Compute`` → sub-steps — that
  ros2_tracing's callback events cannot, since the RT loop is a raw
  ``clock_nanosleep`` thread rather than an rclcpp callback. Absent unless
  the run was built with ``-DRTC_ENABLE_TRACING=ON`` (see docs/tracing.md).
* ``sched_switch`` — emits ``E`` for the prev_tid and ``B`` for the
  next_tid on the corresponding **Cpu** lane, so each lane shows a
  contiguous timeline of "who ran here." Threads not in ``Threads``
  metadata get a synthetic name.
* ``irq_handler_entry`` / ``irq_handler_exit`` — emits ``B`` / ``E`` on
  a special ``Cpu/IRQ`` lane (CPU_PID + 1) so IRQ time is visually
  separable from user threads.

Every other event type is **dropped by default** to keep the JSON small —
the prior behaviour of emitting every unknown tracepoint as an instant
marker (``ph: i``) doubled trace size with content that Perfetto could
not aggregate. Pass ``--keep-events name[,name...]`` to opt specific
event names back in (they re-appear as instant markers on the owning
thread lane). ``--keep-all`` restores the legacy "emit everything"
behaviour for one-off debugging.

Parser: prefers ``python3-bt2`` (the LTTng Python binding) when
importable, falls back to parsing the text output of ``babeltrace2 <dir>``
otherwise. The CLI fallback is slower (regex on each line) but works on
hosts where ``apt install python3-bt2`` has not been run.

Usage::

    python -m rtc_tools.conversion.ctf_to_chrome_trace \
        --input logging_data/260520_1430/tracing/trace --output trace.json
    # Or pipe babeltrace2 yourself:
    babeltrace2 logging_data/260520_1430/tracing/trace \
        | python -m rtc_tools.conversion.ctf_to_chrome_trace --stdin --output trace.json
"""

from __future__ import annotations

import argparse
import contextlib
import json
import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

# Thread-lane slices are grouped by their *real* process (vpid from the
# LTTng DEFAULT_CONTEXT) so Perfetto shows a process → thread hierarchy.
# THREAD_PID is the flat fallback group for captures without the vpid
# context. CPU / IRQ remain virtual groups; their ids sit far above
# PID_MAX_LIMIT (2^22) so they can never collide with a real vpid.
THREAD_PID = 1
CPU_PID = 10_000_000
IRQ_PID = 10_000_001

# process_sort_index values — Perfetto sorts process groups ascending, so
# workspace (rtc:span-emitting) processes render on top, then the CPU
# lanes, then summarized external processes, then IRQ.
SORT_FOCUS = 10
SORT_FALLBACK = 20
SORT_CPU = 30
SORT_SUMMARY = 40
SORT_IRQ = 50

# Events that always produce structured slices (B/E pairs) and are never
# subject to the drop policy — dropping them would empty the timeline.
STRUCTURED_EVENTS = frozenset(
    {
        "ros2:callback_start",
        "ros2:callback_end",
        "sched_switch",
        "irq_handler_entry",
        "irq_handler_exit",
        # rtc_base RTC_TRACE_SCOPE spans — nested B/E on the emitting thread
        # lane, labelled from the begin event's `name` field. These surface
        # RT-tick internals (control loop → controller Compute → sub-steps)
        # that ros2_tracing cannot see, because the RT loop is a raw
        # clock_nanosleep thread, not an rclcpp callback.
        "rtc:span_begin",
        "rtc:span_end",
    }
)

# Events consumed for side-band metadata (never emitted as slices, never
# counted as dropped). rclcpp_callback_register carries the callback
# address → function-symbol mapping used to label callback slices.
METADATA_EVENTS = frozenset({"ros2:rclcpp_callback_register"})

# Events whose ``cpu_id`` (packet context) is actually consumed: only the
# Cpu / Cpu-IRQ lanes read it. Fetching the packet context field for every
# event is a large slice of parse time (a bt2 wrapper object per access),
# so the parser skips it for everything else — the Cpu lane is fully
# reconstructed from sched_switch, which fires on every scheduled CPU.
CPU_LANE_EVENTS = frozenset({"sched_switch", "irq_handler_entry", "irq_handler_exit"})


def _truncate(label: str, limit: int = 80) -> str:
    return label if len(label) <= limit else label[: limit - 3] + "..."


# ─────────────────────────────────────────────────────────────────────────────
# Parser — prefers bt2 binding, falls back to babeltrace2 CLI text parsing
# ─────────────────────────────────────────────────────────────────────────────


def _try_parse_bt2(trace_dir: Path, needed_names: frozenset[str] | None = None):
    """Yield ``(time_ns, event_name, payload_dict, cpu_id_or_None)`` via bt2 binding.

    Returns ``None`` when ``bt2`` cannot be imported, signalling the caller
    to fall back to the CLI parser.

    ``needed_names`` is an optional whitelist of event names whose payload
    the consumer will actually read. For any event *not* in the set the
    payload dict is left empty — dropped events reach ``build_trace`` only
    to be counted, so materialising their fields (a bt2 wrapper object per
    field) is pure waste. ``None`` (the default) disables the filter and
    parses every field of every event, preserving the legacy firehose used
    by ``--keep-all``. ``ts_ns`` and ``name`` are always produced so the
    time origin and drop tally stay identical to the unfiltered walk.
    """
    try:
        import bt2  # type: ignore[import-not-found]
    except ImportError:
        return None

    # Recognise bt2 integer field types by class name so we can coerce
    # them to int even on jazzy/8.x where the .value attribute is absent
    # (hasattr returns False for _SignedIntegerFieldConst and friends).
    _INT_FIELD_TYPENAMES = (
        "_SignedIntegerFieldConst",
        "_UnsignedIntegerFieldConst",
        "_SignedEnumerationFieldConst",
        "_UnsignedEnumerationFieldConst",
    )

    def _to_py(field):
        """Convert a bt2 field object to a JSON-friendly Python primitive.

        bt2 field objects (_SignedIntegerFieldConst, _StringFieldConst, ...)
        are not directly JSON-serializable; trying to dump them blows up
        inside json.encoder. We try the documented ``.value`` attribute
        first, then sniff the class name for known integer types (jazzy
        8.x drops ``.value`` on integer field consts and only str() works
        — but str() returns the numeric *string*, which silently corrupts
        downstream dict-keying-by-tid). Final fallback is str().
        """
        if field is None:
            return None
        if hasattr(field, "value"):
            try:
                return field.value
            except Exception:  # noqa: BLE001
                pass
        # bt2 8.x integer fields lack .value but coerce cleanly via int(str(field)).
        if type(field).__name__ in _INT_FIELD_TYPENAMES:
            try:
                return int(str(field))
            except (TypeError, ValueError):
                pass
        try:
            return str(field)
        except Exception:  # noqa: BLE001
            return None

    def _gen():
        # Auto-discover plugins (src.ctf.fs reads LTTng CTF directories).
        # bt2.TraceCollectionMessageIterator() with a path argument resolves
        # to ctf.fs automatically.
        for msg in bt2.TraceCollectionMessageIterator(str(trace_dir)):
            if not isinstance(msg, bt2._EventMessageConst):  # pylint: disable=protected-access
                continue
            ev = msg.event
            name = ev.name
            ts_ns = msg.default_clock_snapshot.ns_from_origin
            # Payload + context fields flattened into one dict for our use.
            # Skip the (expensive) field materialisation for events the
            # consumer will drop — they only need name + ts to be counted.
            payload: dict = {}
            if needed_names is None or name in needed_names:
                for field_container in (
                    ev.payload_field,
                    ev.specific_context_field,
                    ev.common_context_field,
                ):
                    if field_container is None:
                        continue
                    try:
                        for k in field_container:
                            with contextlib.suppress(Exception):
                                payload[k] = _to_py(field_container[k])
                    except TypeError:
                        pass
            cpu_id = None
            # Only the Cpu / IRQ lanes read cpu_id; fetching the packet
            # context for every event is a large slice of parse time.
            if needed_names is None or name in CPU_LANE_EVENTS:
                try:
                    # CPU id lives in the packet context for LTTng kernel/UST.
                    pctx = ev.packet.context_field
                    if pctx is not None and "cpu_id" in pctx:
                        cpu_id = int(_to_py(pctx["cpu_id"]))
                except Exception:  # noqa: BLE001
                    pass
            yield ts_ns, name, payload, cpu_id

    return _gen()


# babeltrace2 CLI text output example (one event per line):
#
# [14:30:42.123456789] (+0.000002112) hostname ros2:callback_end:
#     { cpu_id = 4 }, { vpid = 123, vtid = 124 }, { callback = 0x7f12... }
#
# Format reference: babeltrace2 default "details" component output.
_BT2_LINE_RE = re.compile(
    r"^\[(?P<ts>[\d:.]+)\]\s+\(\+(?P<delta>[\d.]+)\)\s+"
    r"(?:\S+\s+)?"  # hostname (optional)
    # UST events are "provider:name" but kernel events (sched_switch,
    # irq_handler_*) have no provider prefix — the colon is optional.
    r"(?P<name>\w+(?::\w+)?):\s*"
    r"(?P<rest>.*)$"
)
_BT2_FIELD_RE = re.compile(r"(\w+)\s*=\s*(0x[0-9a-fA-F]+|-?\d+|\"[^\"]*\"|\w+)")


def _parse_bt2_cli_line(
    line: str, base_ns: int | None, needed_names: frozenset[str] | None = None
) -> tuple | None:
    m = _BT2_LINE_RE.match(line)
    if not m:
        return None
    name = m["name"]
    # Convert HH:MM:SS.NNNNNNNNN → ns. We only care about *relative* offsets
    # for the trace; the first event becomes t=0 (ts - base_ns).
    h, mi, s = m["ts"].split(":")
    sec_total = int(h) * 3600 + int(mi) * 60 + float(s)
    ts_ns_abs = int(round(sec_total * 1_000_000_000))
    payload: dict = {}
    cpu_id = None
    # Skip the field regex for events the consumer will drop — same policy
    # as the bt2 path; ts + name still flow so origin and drop tally match.
    if needed_names is None or name in needed_names or name in CPU_LANE_EVENTS:
        for k, v in _BT2_FIELD_RE.findall(m["rest"]):
            if v.startswith("0x"):
                payload[k] = int(v, 16)
            elif v.startswith('"'):
                payload[k] = v.strip('"')
            else:
                try:
                    payload[k] = int(v)
                except ValueError:
                    payload[k] = v
        cpu_id = payload.pop("cpu_id", None)
        # A dropped event whose fields were parsed only for cpu_id must not
        # leak its payload downstream (keeps output identical to bt2 path).
        if needed_names is not None and name not in needed_names:
            payload = {}
    return ts_ns_abs, name, payload, cpu_id, base_ns


def _parse_bt2_cli(stream, needed_names: frozenset[str] | None = None):
    """Yield ``(time_ns, event_name, payload_dict, cpu_id_or_None)`` from babeltrace2 stdout."""
    base_ns: int | None = None
    for line in stream:
        parsed = _parse_bt2_cli_line(line, base_ns, needed_names)
        if parsed is None:
            continue
        ts_ns_abs, name, payload, cpu_id, base_ns = parsed
        if base_ns is None:
            base_ns = ts_ns_abs
        yield ts_ns_abs, name, payload, cpu_id


def iter_events(
    trace_dir: Path | None,
    stdin: bool,
    needed_names: frozenset[str] | None = None,
):
    """Dispatch to bt2 binding if available, else babeltrace2 CLI.

    Announces the chosen parser on stderr so the user knows which path is
    active — the CLI fallback is markedly slower, and that explains a long
    silent parse on hosts without ``python3-bt2``.

    ``needed_names`` (see :func:`_try_parse_bt2`) restricts payload
    materialisation to events the consumer reads; ``None`` parses
    everything. Both parser paths honour it identically.
    """
    if stdin:
        print("[ctf_to_chrome] parser: babeltrace2 CLI text (stdin)", file=sys.stderr)
        return _parse_bt2_cli(sys.stdin, needed_names)
    assert trace_dir is not None
    gen = _try_parse_bt2(trace_dir, needed_names)
    if gen is not None:
        print("[ctf_to_chrome] parser: python3-bt2 binding", file=sys.stderr)
        return gen
    # CLI fallback.
    print(
        "[ctf_to_chrome] parser: babeltrace2 CLI (slower; "
        "'apt install python3-bt2' for the faster binding)",
        file=sys.stderr,
    )
    proc = subprocess.Popen(
        ["babeltrace2", str(trace_dir)],
        stdout=subprocess.PIPE,
        stderr=subprocess.DEVNULL,
        text=True,
    )
    assert proc.stdout is not None
    return _parse_bt2_cli(proc.stdout, needed_names)


# ─────────────────────────────────────────────────────────────────────────────
# Trace builder
# ─────────────────────────────────────────────────────────────────────────────


def build_trace(
    events_iter,
    *,
    keep_events: frozenset[str] | None = None,
    keep_all: bool = False,
    progress_every: int = 0,
    focus_procs: frozenset[str] | None = None,
    all_threads: bool = False,
    max_duration_s: float | None = None,
) -> dict:
    """Walk the event stream once, emit Chrome Trace events.

    Maintains per-CPU "currently-running" tid so sched_switch produces a
    closed B/E pair on the Cpu lane every time a thread is scheduled, and
    an address→symbol map (fed by ``ros2:rclcpp_callback_register``) so
    callback slices are labelled with their function symbol. Callback B/E
    and IRQ B/E are 1:1 with their tracepoints, no further state needed.

    Args:
        events_iter: yields ``(ts_ns, name, payload, cpu_id)``.
        keep_events: extra event names (beyond :data:`STRUCTURED_EVENTS`)
            to emit as instant markers on the owning thread lane. Names
            not in ``STRUCTURED_EVENTS | keep_events`` are silently
            dropped and counted in the returned summary.
        keep_all: if True, emit every non-structured event as an instant
            marker (legacy behaviour, useful for debugging which
            tracepoints fired). Overrides ``keep_events``.
        progress_every: if > 0, print a running "parsed N events" line to
            stderr every ``progress_every`` input events. Large CTF traces
            take tens of seconds to walk with no other output; this shows
            the pass is alive. 0 (the default, used by tests) stays silent.
        focus_procs: process-name substrings forced into the focus tier
            (per-thread detail). Useful for captures without ``rtc:*``
            spans, where the automatic focus classification has no signal.
        all_threads: if True, disable the external-process summarization —
            every UST-emitting process keeps per-thread rows.
        max_duration_s: stop converting this many seconds after the first
            event. Chrome-Trace JSON is a bulky format and the viewer holds
            the whole file in a browser tab, so a long kernel-event capture
            can exceed what it will load; a window keeps every lane intact
            and shrinks the output linearly. Slices still open at the cutoff
            are left unclosed, exactly as at a natural end-of-trace.

    Focus tiering: processes that emitted at least one ``rtc:span_begin``
    (i.e. carry workspace ``RTC_TRACE_SCOPE`` instrumentation) keep full
    per-thread rows. Other UST processes (external ROS nodes such as a
    vendor driver) are collapsed into one async "summary" lane per process
    — their callback slices survive but no longer occupy one row per tid.
    Threads visible only through ``sched_switch`` get no thread row at all
    (they still appear on the Cpu lanes). If no span was captured and no
    ``focus_procs`` match, everything stays at per-thread detail.

    Returns:
        ``{"traceEvents": [...], "displayTimeUnit": "ms",
           "_dropped_event_counts": {name: count, ...},
           "_census": {event_name: count, ..., "_with_vpid": count}}``. The
        underscore keys are consumed by ``main()`` for the stderr summary
        and are stripped before JSON serialization.
    """
    keep_extra = keep_events or frozenset()
    thread_names: dict[int, str] = {}
    symbol_by_addr: dict[int, str] = {}
    cpus_seen: set[int] = set()
    irqs_seen: set[int] = set()
    out: list[dict] = []
    base_ns: int | None = None
    dropped_counts: dict[str, int] = defaultdict(int)

    # Real-process grouping + focus tiering state. tid→pid comes from the
    # vpid/vtid context (ros2_tracing DEFAULT_CONTEXT, present on both UST
    # and kernel channels when enabled at capture).
    tid_to_pid: dict[int, int] = {}
    proc_name_by_pid: dict[int, str] = {}
    span_pids: set[int] = set()  # pids that emitted rtc:span_begin
    thread_slice_tids_by_pid: dict[int, set[int]] = defaultdict(set)

    # Census of the input stream, used by main() to explain an empty or
    # partial timeline. A trace missing rtc:* spans or sched_switch renders
    # as "the group is there but has nothing in it", which is impossible to
    # tell apart from a converter bug without these counts.
    census: dict[str, int] = defaultdict(int)

    # Per-CPU current tid (for sched_switch B/E emission).
    current_tid_on_cpu: dict[int, int] = {}

    # Open callbacks by (tid, callback) so we close the right one.
    callbacks_open: dict[tuple[int, int], int] = {}  # value = start_ts_us

    max_duration_us = None if max_duration_s is None else int(max_duration_s * 1_000_000)
    truncated_at_us: int | None = None

    n_events = 0
    for ts_ns, name, payload, cpu_id in events_iter:
        n_events += 1
        census[name] += 1
        if progress_every and n_events % progress_every == 0:
            print(
                f"[ctf_to_chrome] parsed {n_events:,} events ({len(out):,} slices so far)...",
                file=sys.stderr,
            )
        if base_ns is None:
            base_ns = ts_ns
        ts_us = (ts_ns - base_ns) // 1_000

        if max_duration_us is not None and ts_us > max_duration_us:
            # Stop at the window edge. Breaking (rather than skipping) also
            # cuts conversion time, which is what makes a window usable as a
            # first look at a multi-minute capture.
            truncated_at_us = ts_us
            n_events -= 1  # this event was counted but not converted
            census[name] -= 1
            break

        # Belt-and-suspenders int coercion: even with the _to_py int-field
        # path, the CLI fallback parser returns strings, so harmonise here
        # — Perfetto and our metadata loop both key by numeric tid.
        try:
            vtid = int(payload.get("vtid") or payload.get("tid") or 0)
        except (TypeError, ValueError):
            vtid = 0
        procname = payload.get("procname") or payload.get("comm")
        # UST-derived procname can be stale: a thread that calls
        # pthread_setname_np("rt_callback", ...) *after* its first UST
        # event still gets logged with the parent main-thread comm
        # ("integrated_rt_c"). We register it provisionally here, but the
        # sched_switch branch below overrides with the kernel's view of
        # prev_comm/next_comm — the OS reads /proc/<tid>/comm fresh on
        # every switch, so it reflects the latest pthread_setname_np.
        if vtid and procname and vtid not in thread_names:
            thread_names[vtid] = f"{procname}-{vtid}"

        try:
            vpid = int(payload.get("vpid") or 0)
        except (TypeError, ValueError):
            vpid = 0
        if vpid:
            census["_with_vpid"] += 1
        if vpid and vtid:
            tid_to_pid[vtid] = vpid
            if procname:
                if vtid == vpid:
                    # Main thread: its comm IS the process name — freshest wins.
                    proc_name_by_pid[vpid] = str(procname)
                else:
                    # Worker-thread comm is a weak guess; keep only until the
                    # main thread is observed (UST context or sched_switch).
                    proc_name_by_pid.setdefault(vpid, str(procname))

        # Thread-lane events land on their real process group when the trace
        # carries the vpid context; THREAD_PID is the flat fallback group
        # for context-less captures (and synthetic test streams).
        lane_pid = vpid or tid_to_pid.get(vtid, 0) or THREAD_PID

        if cpu_id is not None:
            cpus_seen.add(int(cpu_id))

        if name == "ros2:rclcpp_callback_register":
            # Side-band metadata: address → function symbol, used to label
            # callback slices below. Consumed, not dropped; the legacy
            # instant-marker emission still applies under keep_all /
            # keep_events so --keep-all remains a faithful firehose.
            try:
                addr = int(payload.get("callback", 0))
            except (TypeError, ValueError):
                addr = 0
            sym = payload.get("symbol")
            if addr and sym:
                symbol_by_addr[addr] = str(sym)
            if keep_all or name in keep_extra:
                thread_slice_tids_by_pid[lane_pid].add(vtid)
                out.append(
                    {
                        "name": _truncate(name),
                        "cat": "ros2",
                        "ph": "i",
                        "ts": ts_us,
                        "pid": lane_pid,
                        "tid": vtid,
                        "s": "t",
                        "args": payload,
                    }
                )

        elif name == "ros2:callback_start":
            cb = int(payload.get("callback", 0))
            sym = payload.get("symbol") or symbol_by_addr.get(cb) or f"callback@0x{cb:x}"
            key = (vtid, cb)
            callbacks_open[key] = ts_us
            thread_slice_tids_by_pid[lane_pid].add(vtid)
            out.append(
                {
                    "name": _truncate(sym),
                    "cat": "callback",
                    "ph": "B",
                    "ts": ts_us,
                    "pid": lane_pid,
                    "tid": vtid,
                    "args": {"callback": f"0x{cb:x}", "symbol": sym},
                }
            )

        elif name == "ros2:callback_end":
            cb = int(payload.get("callback", 0))
            key = (vtid, cb)
            if key in callbacks_open:
                del callbacks_open[key]
            out.append(
                {
                    "ph": "E",
                    "ts": ts_us,
                    "pid": lane_pid,
                    "tid": vtid,
                    "cat": "callback",
                }
            )

        elif name == "rtc:span_begin":
            # RAII scope entered (RTC_TRACE_SCOPE). The `name` field is a
            # compile-time literal (e.g. "DemoWbcController::ComputeControl").
            # Perfetto stacks consecutive B/E on the same (pid, tid) lane, and
            # the RAII helper guarantees balanced nesting, so no per-tid stack
            # bookkeeping is needed here.
            label = payload.get("name") or "rtc:span"
            span_pids.add(lane_pid)
            thread_slice_tids_by_pid[lane_pid].add(vtid)
            out.append(
                {
                    "name": _truncate(str(label)),
                    "cat": "rtc",
                    "ph": "B",
                    "ts": ts_us,
                    "pid": lane_pid,
                    "tid": vtid,
                }
            )

        elif name == "rtc:span_end":
            out.append(
                {
                    "ph": "E",
                    "ts": ts_us,
                    "pid": lane_pid,
                    "tid": vtid,
                    "cat": "rtc",
                }
            )

        elif name == "sched_switch":
            prev_tid = int(payload.get("prev_tid", 0))
            next_tid = int(payload.get("next_tid", 0))
            prev_comm = payload.get("prev_comm") or ""
            next_comm = payload.get("next_comm") or ""
            # The kernel's prev_comm/next_comm is the freshest signal we
            # get for a thread's name: /proc/<tid>/comm is re-read on
            # every sched_switch, so any pthread_setname_np that has run
            # by this point is reflected. UST registrations above may
            # have used a stale parent-thread comm; overwrite them.
            if prev_tid and prev_comm:
                thread_names[prev_tid] = f"{prev_comm}-{prev_tid}"
            if next_tid and next_comm:
                thread_names[next_tid] = f"{next_comm}-{next_tid}"
            # Refresh the process name whenever the *main* thread (tid == pid)
            # is switched in/out — its comm is the process name, fresher than
            # any weak worker-thread guess taken from the UST context.
            for t, comm in ((prev_tid, prev_comm), (next_tid, next_comm)):
                if t and comm and tid_to_pid.get(t) == t:
                    proc_name_by_pid[t] = comm
            if cpu_id is None:
                continue
            cpu = int(cpu_id)
            cur = current_tid_on_cpu.get(cpu)
            if cur is not None:
                out.append(
                    {
                        "ph": "E",
                        "ts": ts_us,
                        "pid": CPU_PID,
                        "tid": cpu,
                        "cat": "run",
                    }
                )
            if next_tid != 0:  # skip swapper/idle slices to keep JSON small
                label = thread_names.get(next_tid, next_comm or f"tid-{next_tid}")
                out.append(
                    {
                        "name": _truncate(label),
                        "cat": "run",
                        "ph": "B",
                        "ts": ts_us,
                        "pid": CPU_PID,
                        "tid": cpu,
                        "args": {"tid": next_tid, "comm": next_comm},
                    }
                )
                current_tid_on_cpu[cpu] = next_tid
            else:
                current_tid_on_cpu.pop(cpu, None)

        elif name == "irq_handler_entry" and cpu_id is not None:
            irq_num = int(payload.get("irq", -1))
            irq_name = payload.get("name") or f"irq-{irq_num}"
            irqs_seen.add(int(cpu_id))
            out.append(
                {
                    "name": _truncate(f"IRQ {irq_name}"),
                    "cat": "irq",
                    "ph": "B",
                    "ts": ts_us,
                    "pid": IRQ_PID,
                    "tid": int(cpu_id),
                    "args": {"irq": irq_num, "name": irq_name},
                }
            )

        elif name == "irq_handler_exit" and cpu_id is not None:
            out.append(
                {
                    "ph": "E",
                    "ts": ts_us,
                    "pid": IRQ_PID,
                    "tid": int(cpu_id),
                    "cat": "irq",
                }
            )

        else:
            # Drop policy: by default everything not in STRUCTURED_EVENTS
            # is silently dropped — emitting them as instant markers
            # bloated the JSON without adding aggregatable signal in
            # Perfetto. Opt-in via --keep-events <name> or --keep-all.
            if keep_all or name in keep_extra:
                thread_slice_tids_by_pid[lane_pid].add(vtid)
                out.append(
                    {
                        "name": _truncate(name),
                        "cat": name.split(":")[0] if ":" in name else "event",
                        "ph": "i",
                        "ts": ts_us,
                        "pid": lane_pid,
                        "tid": vtid,
                        "s": "t",
                        "args": payload,
                    }
                )
            else:
                dropped_counts[name] += 1

    # ── Post-pass 1: focus/summary tier classification ───────────────────────
    # Focus = processes carrying workspace RTC_TRACE_SCOPE instrumentation
    # (data-driven — no process-name hardcoding). The THREAD_PID fallback
    # group (context-less captures) is always kept at per-thread detail.
    ust_pids = set(thread_slice_tids_by_pid) - {THREAD_PID}
    if all_threads:
        focus_pids = set(ust_pids)
    else:
        focus_pids = (span_pids - {THREAD_PID}) & ust_pids
        if focus_procs:
            for pid in ust_pids:
                nm = proc_name_by_pid.get(pid, "")
                if any(s in nm for s in focus_procs):
                    focus_pids.add(pid)
        if not focus_pids and ust_pids:
            print(
                "[ctf_to_chrome] no rtc:* spans captured and no --focus-proc "
                "match — keeping all processes at per-thread detail "
                "(focus tiering needs an RTC_ENABLE_TRACING=ON build, see "
                "docs/tracing.md).",
                file=sys.stderr,
            )
            focus_pids = set(ust_pids)
    summary_pids = ust_pids - focus_pids

    # ── Post-pass 2: collapse summary processes into async lanes ────────────
    # Sync B/E slices from different tids of one process overlap in time, so
    # merging them onto one lane needs Chrome async events (ph b/e + id):
    # Perfetto renders them as a single process-level track that only forks
    # sub-rows while slices actually overlap.
    if summary_pids:
        open_stacks: dict[tuple[int, int], list[tuple[str, str, str]]] = defaultdict(list)
        async_seq = 0
        orphan_idx: set[int] = set()
        for idx, ev in enumerate(out):
            if ev.get("pid") not in summary_pids:
                continue
            ph = ev.get("ph")
            if ph == "B":
                async_seq += 1
                aid = f"{ev['pid']}.{async_seq}"
                ev["ph"] = "b"
                ev["id"] = aid
                open_stacks[(ev["pid"], ev["tid"])].append((aid, ev["name"], ev.get("cat", "")))
            elif ph == "E":
                stack = open_stacks[(ev["pid"], ev["tid"])]
                if stack:
                    aid, nm, cat = stack.pop()
                    ev["ph"] = "e"
                    ev["id"] = aid
                    ev["name"] = nm
                    ev["cat"] = cat
                else:
                    # End with no captured begin (trace started mid-slice) —
                    # an unmatched async "e" confuses Perfetto; drop it.
                    orphan_idx.add(idx)
        if orphan_idx:
            out = [ev for idx, ev in enumerate(out) if idx not in orphan_idx]

    # ── Post-pass 3: CPU-lane labels get a process prefix ────────────────────
    # sched_switch only carries comm; prefix the owning process name where
    # the tid→pid mapping is known (main threads keep their bare comm — a
    # "rt_control/rt_control-123" prefix would be noise).
    for ev in out:
        if ev.get("pid") != CPU_PID or ev.get("ph") != "B":
            continue
        t = ev["args"].get("tid")
        base = thread_names.get(t) or ev["name"]
        pid = tid_to_pid.get(t)
        proc = proc_name_by_pid.get(pid) if pid else None
        if proc and not base.startswith(f"{proc}-"):
            ev["name"] = _truncate(f"{proc}/{base}")
        else:
            ev["name"] = _truncate(base)

    # ── Metadata records so Perfetto labels and orders the swimlanes ─────────
    metadata: list[dict] = []

    def _process_meta(pid: int, label: str, sort_index: int) -> None:
        metadata.append(
            {"name": "process_name", "ph": "M", "pid": pid, "tid": 0, "args": {"name": label}}
        )
        metadata.append(
            {
                "name": "process_sort_index",
                "ph": "M",
                "pid": pid,
                "tid": 0,
                "args": {"sort_index": sort_index},
            }
        )

    def _thread_meta(pid: int, tid: int) -> None:
        metadata.append(
            {
                "name": "thread_name",
                "ph": "M",
                "pid": pid,
                "tid": tid,
                "args": {"name": thread_names.get(tid, f"tid-{tid}")},
            }
        )

    for pid in sorted(focus_pids):
        _process_meta(pid, proc_name_by_pid.get(pid) or f"pid-{pid}", SORT_FOCUS)
        for tid in sorted(thread_slice_tids_by_pid.get(pid, ())):
            _thread_meta(pid, tid)
    if THREAD_PID in thread_slice_tids_by_pid:
        _process_meta(THREAD_PID, "Threads (by TID)", SORT_FALLBACK)
        for tid in sorted(thread_slice_tids_by_pid[THREAD_PID]):
            _thread_meta(THREAD_PID, tid)
    # Threads seen only via sched_switch get no thread row — they already
    # show on the Cpu lanes, and one empty row per system thread is what
    # drowned the workspace threads in the old flat layout.
    #
    # Only advertise the Cpus group when sched_switch actually produced
    # lanes: an empty "Cpus" group is indistinguishable from a broken
    # converter, when the real cause is a capture without kernel events.
    if cpus_seen:
        _process_meta(CPU_PID, "Cpus", SORT_CPU)
    for pid in sorted(summary_pids):
        label = proc_name_by_pid.get(pid) or f"pid-{pid}"
        _process_meta(pid, f"{label} (summary)", SORT_SUMMARY)
    if irqs_seen:
        _process_meta(IRQ_PID, "Cpu/IRQ", SORT_IRQ)
    for cpu in sorted(cpus_seen):
        metadata.append(
            {
                "name": "thread_name",
                "ph": "M",
                "pid": CPU_PID,
                "tid": cpu,
                "args": {"name": f"Cpu {cpu:03d}"},
            }
        )
        if cpu in irqs_seen:
            metadata.append(
                {
                    "name": "thread_name",
                    "ph": "M",
                    "pid": IRQ_PID,
                    "tid": cpu,
                    "args": {"name": f"Cpu {cpu:03d} IRQ"},
                }
            )

    if summary_pids:
        summarized = ", ".join(
            f"{proc_name_by_pid.get(p) or f'pid-{p}'}({p})" for p in sorted(summary_pids)
        )
        print(
            f"[ctf_to_chrome] summarized {len(summary_pids)} external process(es) "
            f"into async lanes: {summarized} (per-thread rows: --all-threads)",
            file=sys.stderr,
        )
    if truncated_at_us is not None:
        print(
            f"[ctf_to_chrome] --max-duration-s: stopped at "
            f"{truncated_at_us / 1_000_000:.3f}s; later events were NOT converted "
            f"(census below covers the window only).",
            file=sys.stderr,
        )

    return {
        "traceEvents": metadata + out,
        "displayTimeUnit": "ms",
        "_dropped_event_counts": dict(dropped_counts),
        "_census": dict(census),
    }


def _report_capture_gaps(census: dict[str, int]) -> list[str]:
    """Print what the *capture* is missing, and return the warning lines.

    An absent event class renders in Perfetto as a swimlane group that is
    simply not there (or is there but empty) — visually identical to a
    converter bug. Each check below names the capture-side cause so the
    trace, not the tool, gets fixed. Returns the emitted lines for tests.
    """
    lines: list[str] = []
    spans = census.get("rtc:span_begin", 0)
    callbacks = census.get("ros2:callback_start", 0)
    switches = census.get("sched_switch", 0)
    irqs = census.get("irq_handler_entry", 0)
    with_vpid = census.get("_with_vpid", 0)

    lines.append(
        f"[ctf_to_chrome] capture census: rtc:span_begin={spans:,} "
        f"ros2:callback_start={callbacks:,} sched_switch={switches:,} "
        f"irq_handler_entry={irqs:,}"
    )
    if not spans:
        lines.append(
            "[ctf_to_chrome] WARNING: 0 rtc:* spans. RT tick internals "
            "(rt_control_tick / sim_step / hand_comm_tick and their phases) "
            "will be ABSENT — the RT loop is a raw clock_nanosleep thread, "
            "not an rclcpp callback, so nothing else can show it. Rebuild "
            "with tracing on ('./build.sh sim --tracing', i.e. "
            "-DRTC_ENABLE_TRACING=ON) and re-capture. See docs/tracing.md."
        )
    if not switches:
        lines.append(
            "[ctf_to_chrome] WARNING: 0 sched_switch. The Cpus swimlanes "
            "(which thread ran on which core, and for how long) CANNOT be "
            "built — that view is reconstructed entirely from sched_switch. "
            "Kernel events were not captured: check 'groups' contains "
            "'tracing' IN THE SHELL THAT LAUNCHED (membership needs a "
            "re-login; 'newgrp tracing'), that lttng-modules is loadable "
            "('lttng list --kernel'), and that trace_events_kernel was not "
            "overridden to empty. See docs/tracing.md §Permissions."
        )
    if callbacks and not with_vpid:
        lines.append(
            "[ctf_to_chrome] WARNING: no vpid context on any event — "
            "per-process grouping is unavailable, falling back to a flat "
            "thread group. Capture with the default context_fields "
            "(procname, vpid, vtid)."
        )
    for line in lines:
        print(line, file=sys.stderr)
    return lines


# Chrome-Trace JSON that Perfetto's legacy importer handles comfortably in
# a browser tab. Measured: a 30 s traced sim run = ~2.9 M events / ~295 MB
# (≈100 B per event), dominated by rtc:* spans + sched_switch.
_JSON_SIZE_WARN_MB = 250


def _report_size_risk(n_events: int, size_mb: float) -> list[str]:
    """Warn when the JSON is large enough for the viewer to struggle.

    Metadata records are emitted *before* the slices, so a viewer that
    gives up partway through a huge file still draws the process/thread
    groups — they just come out empty. That failure is indistinguishable
    from "the converter produced nothing", so name it explicitly.
    """
    if size_mb < _JSON_SIZE_WARN_MB:
        return []
    lines = [
        f"[ctf_to_chrome] WARNING: {size_mb:,.0f} MB / {n_events:,} events is large "
        "for Perfetto's JSON importer. If the UI shows the process groups but "
        "their lanes look EMPTY, the viewer ran out of room part-way — the "
        "group labels are metadata at the top of the file, the slices come "
        "after. Narrow the capture rather than the view:",
        "[ctf_to_chrome]   - capture for less wall-clock time (size is linear in it)",
        "[ctf_to_chrome]   - drop kernel events if you only need rtc:* spans "
        "(trace_events_kernel:=), or narrow trace_events_ust if you only need the Cpus view",
        "[ctf_to_chrome]   - keep only the processes you care about (--focus-proc)",
    ]
    for line in lines:
        print(line, file=sys.stderr)
    return lines


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument(
        "--input",
        "-i",
        type=Path,
        help="CTF trace directory (e.g. ~/.ros/tracing/260520_1430).",
    )
    src.add_argument(
        "--stdin",
        action="store_true",
        help="Read babeltrace2 text output from stdin (skip subprocess).",
    )
    p.add_argument(
        "--output",
        "-o",
        type=Path,
        required=True,
        help="Output Chrome Trace JSON path.",
    )
    p.add_argument(
        "--keep-events",
        default="",
        help=(
            "Comma-separated event names (e.g. 'ros2:rclcpp_publish,"
            "sched_wakeup') to retain as instant markers in addition to "
            "the structured B/E set. Default: drop everything else."
        ),
    )
    p.add_argument(
        "--keep-all",
        action="store_true",
        help=(
            "Legacy behaviour: emit every non-structured event as an "
            "instant marker. Useful when investigating which tracepoints "
            "fired, but produces large JSON."
        ),
    )
    p.add_argument(
        "--all-threads",
        action="store_true",
        help=(
            "Disable the external-process summarization: every UST-emitting "
            "process keeps per-thread rows (default: processes without "
            "rtc:* spans collapse into one async summary lane each)."
        ),
    )
    p.add_argument(
        "--focus-proc",
        default="",
        help=(
            "Comma-separated process-name substrings to force into the "
            "focus tier (per-thread rows). Useful for captures from a "
            "build without RTC_ENABLE_TRACING, where no rtc:* span exists "
            "to classify workspace processes automatically."
        ),
    )
    p.add_argument(
        "--max-duration-s",
        type=float,
        default=None,
        help=(
            "Convert only the first N seconds of the trace. Chrome-Trace "
            "JSON is bulky and the viewer holds the whole file in a browser "
            "tab; a window keeps every lane intact and shrinks the output "
            "(and the conversion time) linearly."
        ),
    )
    args = p.parse_args(argv)
    keep_events = frozenset(item.strip() for item in args.keep_events.split(",") if item.strip())
    focus_procs = frozenset(item.strip() for item in args.focus_proc.split(",") if item.strip())
    if args.max_duration_s is not None and args.max_duration_s <= 0:
        print("[ctf_to_chrome] --max-duration-s must be > 0", file=sys.stderr)
        return 2

    if args.input is not None and not args.input.is_dir():
        print(f"[ctf_to_chrome] input not a directory: {args.input}", file=sys.stderr)
        return 1

    # Only these events' payloads are read downstream; let the parser skip
    # field materialisation for the rest (large parse-time win on big
    # traces). --keep-all needs every payload, so disable the filter then.
    needed_names = None if args.keep_all else STRUCTURED_EVENTS | METADATA_EVENTS | keep_events
    events_iter = iter_events(args.input, args.stdin, needed_names)
    print("[ctf_to_chrome] parsing events...", file=sys.stderr)
    trace = build_trace(
        events_iter,
        keep_events=keep_events,
        keep_all=args.keep_all,
        progress_every=100_000,
        focus_procs=focus_procs,
        all_threads=args.all_threads,
        max_duration_s=args.max_duration_s,
    )

    # Strip the in-process sentinel before JSON dump (Chrome Trace
    # spec rejects unknown top-level keys with leading underscore on
    # some viewers; safer to drop).
    dropped_counts = trace.pop("_dropped_event_counts", {})
    census = trace.pop("_census", {})
    _report_capture_gaps(census)

    if not any(ev["ph"] != "M" for ev in trace["traceEvents"]):
        print(
            "[ctf_to_chrome] no events parsed — empty trace or unrecognised CTF format.\n"
            "  Check: ls <input> (should contain 'metadata' and channel files)\n"
            "  Check: babeltrace2 <input> | head  (should print readable events)",
            file=sys.stderr,
        )
        return 1

    print(
        f"[ctf_to_chrome] writing {len(trace['traceEvents']):,} events → {args.output}",
        file=sys.stderr,
    )
    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w") as f:
        json.dump(trace, f)

    by_phase: dict[str, int] = defaultdict(int)
    for ev in trace["traceEvents"]:
        by_phase[ev["ph"]] += 1
    phase_str = "/".join(f"{count}{ph}" for ph, count in sorted(by_phase.items()))
    print(
        f"[ctf_to_chrome] wrote {args.output} ({len(trace['traceEvents'])} events: {phase_str})",
        file=sys.stderr,
    )
    _report_size_risk(len(trace["traceEvents"]), args.output.stat().st_size / (1024 * 1024))
    if dropped_counts:
        total_dropped = sum(dropped_counts.values())
        # Show the top contributors so the user can decide what to add
        # back via --keep-events. Cap at 5 entries to keep stderr tidy.
        top = sorted(dropped_counts.items(), key=lambda kv: kv[1], reverse=True)[:5]
        top_str = ", ".join(f"{n}={c}" for n, c in top)
        print(
            f"[ctf_to_chrome] dropped {total_dropped} non-structured events "
            f"({len(dropped_counts)} distinct names). Top: {top_str}.",
            file=sys.stderr,
        )
        print(
            "[ctf_to_chrome]   add specific names back with "
            "--keep-events <comma-list>, or --keep-all for everything.",
            file=sys.stderr,
        )
    print(
        "[ctf_to_chrome] open https://ui.perfetto.dev and drag-drop the JSON",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
