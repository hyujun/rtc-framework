"""Convert an LTTng CTF trace into a Chrome Trace JSON for Perfetto UI.

Input: a directory containing an LTTng CTF trace (the output of
``ros2 launch ... enable_tracing:=true`` lives at
``<ws>/logging_data/<YYMMDD_HHMM>/tracing/<session_name>/``; manual
``ros2 trace`` CLI captures land under ``~/.ros/tracing/``). The directory
is expected to contain ``metadata`` plus one or more channel binaries —
babeltrace2 walks any sub-tree automatically.

Output: a Chrome Trace JSON file. Drag-drop it onto
https://ui.perfetto.dev to see two swimlane groups simultaneously:

* **Threads (by TID)** — each callback / sched_switch slice plotted per
  task. Use this to inspect *what each thread was doing* over time.
* **Cpus** — the same data resorted by ``cpu_id``. Use this to verify
  ApplyThreadConfig pinning (Core 2 = rt_control, MPC core, etc.) and
  spot migrations or IRQ leaks.

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

# Two virtual processes so Perfetto draws two swimlane groups.
THREAD_PID = 1
CPU_PID = 2
IRQ_PID = 3

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


def _truncate(label: str, limit: int = 80) -> str:
    return label if len(label) <= limit else label[: limit - 3] + "..."


# ─────────────────────────────────────────────────────────────────────────────
# Parser — prefers bt2 binding, falls back to babeltrace2 CLI text parsing
# ─────────────────────────────────────────────────────────────────────────────


def _try_parse_bt2(trace_dir: Path):
    """Yield ``(time_ns, event_name, payload_dict, cpu_id_or_None)`` via bt2 binding.

    Returns ``None`` when ``bt2`` cannot be imported, signalling the caller
    to fall back to the CLI parser.
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
            ts_ns = msg.default_clock_snapshot.ns_from_origin
            # Payload + context fields flattened into one dict for our use.
            payload: dict = {}
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
            try:
                # CPU id lives in the packet context for LTTng kernel/UST.
                pctx = ev.packet.context_field
                if pctx is not None and "cpu_id" in pctx:
                    cpu_id = int(_to_py(pctx["cpu_id"]))
            except Exception:  # noqa: BLE001
                pass
            yield ts_ns, ev.name, payload, cpu_id

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


def _parse_bt2_cli_line(line: str, base_ns: int | None) -> tuple | None:
    m = _BT2_LINE_RE.match(line)
    if not m:
        return None
    # Convert HH:MM:SS.NNNNNNNNN → ns. We only care about *relative* offsets
    # for the trace; the first event becomes t=0 (ts - base_ns).
    h, mi, s = m["ts"].split(":")
    sec_total = int(h) * 3600 + int(mi) * 60 + float(s)
    ts_ns_abs = int(round(sec_total * 1_000_000_000))
    payload: dict = {}
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
    return ts_ns_abs, m["name"], payload, cpu_id, base_ns


def _parse_bt2_cli(stream):
    """Yield ``(time_ns, event_name, payload_dict, cpu_id_or_None)`` from babeltrace2 stdout."""
    base_ns: int | None = None
    for line in stream:
        parsed = _parse_bt2_cli_line(line, base_ns)
        if parsed is None:
            continue
        ts_ns_abs, name, payload, cpu_id, base_ns = parsed
        if base_ns is None:
            base_ns = ts_ns_abs
        yield ts_ns_abs, name, payload, cpu_id


def iter_events(trace_dir: Path | None, stdin: bool):
    """Dispatch to bt2 binding if available, else babeltrace2 CLI.

    Announces the chosen parser on stderr so the user knows which path is
    active — the CLI fallback is markedly slower, and that explains a long
    silent parse on hosts without ``python3-bt2``.
    """
    if stdin:
        print("[ctf_to_chrome] parser: babeltrace2 CLI text (stdin)", file=sys.stderr)
        return _parse_bt2_cli(sys.stdin)
    assert trace_dir is not None
    gen = _try_parse_bt2(trace_dir)
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
    return _parse_bt2_cli(proc.stdout)


# ─────────────────────────────────────────────────────────────────────────────
# Trace builder
# ─────────────────────────────────────────────────────────────────────────────


def build_trace(
    events_iter,
    *,
    keep_events: frozenset[str] | None = None,
    keep_all: bool = False,
    progress_every: int = 0,
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

    Returns:
        ``{"traceEvents": [...], "displayTimeUnit": "ms",
           "_dropped_event_counts": {name: count, ...}}``. The
        ``_dropped_event_counts`` key is consumed by ``main()`` for the
        stderr summary and is stripped before JSON serialization.
    """
    keep_extra = keep_events or frozenset()
    thread_names: dict[int, str] = {}
    symbol_by_addr: dict[int, str] = {}
    cpus_seen: set[int] = set()
    irqs_seen: set[int] = set()
    out: list[dict] = []
    base_ns: int | None = None
    dropped_counts: dict[str, int] = defaultdict(int)

    # Per-CPU current tid (for sched_switch B/E emission).
    current_tid_on_cpu: dict[int, int] = {}

    # Open callbacks by (tid, callback) so we close the right one.
    callbacks_open: dict[tuple[int, int], int] = {}  # value = start_ts_us

    n_events = 0
    for ts_ns, name, payload, cpu_id in events_iter:
        n_events += 1
        if progress_every and n_events % progress_every == 0:
            print(
                f"[ctf_to_chrome] parsed {n_events:,} events ({len(out):,} slices so far)...",
                file=sys.stderr,
            )
        if base_ns is None:
            base_ns = ts_ns
        ts_us = (ts_ns - base_ns) // 1_000

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
                out.append(
                    {
                        "name": _truncate(name),
                        "cat": "ros2",
                        "ph": "i",
                        "ts": ts_us,
                        "pid": THREAD_PID,
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
            out.append(
                {
                    "name": _truncate(sym),
                    "cat": "callback",
                    "ph": "B",
                    "ts": ts_us,
                    "pid": THREAD_PID,
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
                    "pid": THREAD_PID,
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
            out.append(
                {
                    "name": _truncate(str(label)),
                    "cat": "rtc",
                    "ph": "B",
                    "ts": ts_us,
                    "pid": THREAD_PID,
                    "tid": vtid,
                }
            )

        elif name == "rtc:span_end":
            out.append(
                {
                    "ph": "E",
                    "ts": ts_us,
                    "pid": THREAD_PID,
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
                out.append(
                    {
                        "name": _truncate(name),
                        "cat": name.split(":")[0] if ":" in name else "event",
                        "ph": "i",
                        "ts": ts_us,
                        "pid": THREAD_PID,
                        "tid": vtid,
                        "s": "t",
                        "args": payload,
                    }
                )
            else:
                dropped_counts[name] += 1

    # Metadata records so Perfetto labels the swimlanes.
    metadata: list[dict] = [
        {
            "name": "process_name",
            "ph": "M",
            "pid": THREAD_PID,
            "tid": 0,
            "args": {"name": "Threads (by TID)"},
        },
        {
            "name": "process_name",
            "ph": "M",
            "pid": CPU_PID,
            "tid": 0,
            "args": {"name": "Cpus"},
        },
    ]
    if irqs_seen:
        metadata.append(
            {
                "name": "process_name",
                "ph": "M",
                "pid": IRQ_PID,
                "tid": 0,
                "args": {"name": "Cpu/IRQ"},
            }
        )
    for tid, label in thread_names.items():
        metadata.append(
            {
                "name": "thread_name",
                "ph": "M",
                "pid": THREAD_PID,
                "tid": tid,
                "args": {"name": label},
            }
        )
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

    return {
        "traceEvents": metadata + out,
        "displayTimeUnit": "ms",
        "_dropped_event_counts": dict(dropped_counts),
    }


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
    args = p.parse_args(argv)
    keep_events = frozenset(item.strip() for item in args.keep_events.split(",") if item.strip())

    if args.input is not None and not args.input.is_dir():
        print(f"[ctf_to_chrome] input not a directory: {args.input}", file=sys.stderr)
        return 1

    events_iter = iter_events(args.input, args.stdin)
    print("[ctf_to_chrome] parsing events...", file=sys.stderr)
    trace = build_trace(
        events_iter,
        keep_events=keep_events,
        keep_all=args.keep_all,
        progress_every=100_000,
    )

    # Strip the in-process sentinel before JSON dump (Chrome Trace
    # spec rejects unknown top-level keys with leading underscore on
    # some viewers; safer to drop).
    dropped_counts = trace.pop("_dropped_event_counts", {})

    if len(trace["traceEvents"]) <= 2:  # only the 2 metadata 'M' records
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
    print(
        f"[ctf_to_chrome] wrote {args.output} "
        f"({len(trace['traceEvents'])} events: "
        f"{by_phase['B']}B/{by_phase['E']}E/{by_phase['i']}I/{by_phase['M']}M)",
        file=sys.stderr,
    )
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
