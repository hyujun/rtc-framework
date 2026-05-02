"""Convert ``perf script`` output into a Chrome Trace JSON for Perfetto UI.

Two output modes:

* ``--mode flamechart`` (default) — for **"who ran when"** views. Adjacent
  samples on the same thread are stitched into begin/end pairs along their
  common callstack prefix, producing horizontal bars labelled with function
  names. This is a *flame chart* (time-axis flame graph), not a flame graph.
* ``--mode instant`` — for **timestamp-precise** views. Each sample becomes
  a single vertical instant marker; hover reveals the captured stack. Use
  when you need exact sample timestamps without interpolation.

Usage::

    perf script -i perf.data | python -m rtc_tools.conversion.perf_to_chrome_trace \
        --output trace.json --max-depth 10
    # or (let the script invoke perf for you):
    python -m rtc_tools.conversion.perf_to_chrome_trace \
        --input perf.data --output trace.json

Then drag-drop ``trace.json`` onto https://ui.perfetto.dev.

Layout: two virtual processes — ``Threads (by TID)`` and ``Cpus`` — each
displaying its members as swimlanes, so you see the same data sorted both
ways simultaneously.

Why a custom converter:

* ``perf data convert --to-ctf`` drops user-space callstacks.
* ``perfetto traced_perf`` only works during a fresh capture, not from an
  existing ``perf.data``.

Sampling caveat:

* perf samples are 99–999 Hz snapshots, **not** begin/end pairs. The
  flamechart mode infers function durations from common-prefix continuity:
  two adjacent samples sharing the first K stack frames mean those K
  frames were live for the entire interval between the two timestamps.
  This is the same approximation Brendan Gregg's flamegraph and Speedscope
  use; it's accurate to within one sample period.
* Off-CPU intervals are NOT captured (would require sched_switch tracepoints
  or ``--off-cpu`` at record time, which our default capture does not enable).
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from collections import defaultdict
from pathlib import Path

# perf script header format:
#   ur5e_rt_control  251293 [009] 160835.363498:        144 cycles:P:
# Capture: comm (greedy, may contain spaces collapsed by perf), pid, cpu, time, period.
# perf left-pads the comm field with spaces so we anchor on the trailing pid.
_HEADER_RE = re.compile(
    r"^(?P<comm>.+?)\s+(?P<pid>\d+)\s+\[(?P<cpu>\d+)\]\s+"
    r"(?P<time>\d+\.\d+):\s+(?P<period>\d+)\s+\S+:"
)

# Stack frame line: tab-indented "address symbol+offset (dso)" or "address [unknown] ([unknown])"
_FRAME_RE = re.compile(r"^\s+[0-9a-fA-F]+\s+(?P<sym>.+?)(?:\s+\((?P<dso>[^)]*)\))?\s*$")


def _strip_offset(symbol: str) -> str:
    """Drop trailing '+0xNN (inlined)' or '+0xNN' so identical symbols collapse."""
    if symbol.endswith(" (inlined)"):
        symbol = symbol[: -len(" (inlined)")]
    plus = symbol.rfind("+0x")
    if plus != -1 and all(c in "0123456789abcdefABCDEF" for c in symbol[plus + 3 :]):
        symbol = symbol[:plus]
    return symbol


def parse_perf_script(stream, *, max_depth: int) -> tuple[list[dict], dict[int, str], list[int]]:
    """Parse a ``perf script`` text stream.

    Returns:
        samples: list of dicts ``{time_us, cpu, tid, comm, frames[]}`` where
            ``frames`` is leaf-first and truncated to ``max_depth``.
        thread_names: mapping ``tid → "comm-pid/tid"``.
        cpus_seen: sorted unique CPU ids encountered.
    """
    samples: list[dict] = []
    thread_names: dict[int, str] = {}
    cpus_seen: set[int] = set()

    current_header: dict | None = None
    current_frames: list[str] = []

    def flush() -> None:
        if current_header is None:
            return
        frames = current_frames[:max_depth] if max_depth > 0 else current_frames
        samples.append({**current_header, "frames": frames})

    for raw in stream:
        line = raw.rstrip("\n")
        if not line:
            flush()
            current_header = None
            current_frames = []
            continue

        m = _HEADER_RE.match(line)
        if m:
            flush()
            tid = int(m["pid"])
            cpu = int(m["cpu"])
            time_us = int(round(float(m["time"]) * 1_000_000))
            comm = m["comm"].strip()
            current_header = {
                "time_us": time_us,
                "cpu": cpu,
                "tid": tid,
                "comm": comm,
            }
            current_frames = []
            thread_names[tid] = f"{comm}-{tid}"
            cpus_seen.add(cpu)
            continue

        m = _FRAME_RE.match(line)
        if m and current_header is not None:
            sym = _strip_offset(m["sym"].strip())
            current_frames.append(sym)

    flush()
    return samples, thread_names, sorted(cpus_seen)


# ─────────────────────────────────────────────────────────────────────────────
# Trace builders
# ─────────────────────────────────────────────────────────────────────────────


THREAD_PID = 1
CPU_PID = 2


def _truncate(label: str, limit: int = 80) -> str:
    return label if len(label) <= limit else label[: limit - 3] + "..."


def _emit_metadata(thread_names: dict[int, str], cpus: list[int]) -> list[dict]:
    events: list[dict] = [
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
    for tid, name in thread_names.items():
        events.append(
            {
                "name": "thread_name",
                "ph": "M",
                "pid": THREAD_PID,
                "tid": tid,
                "args": {"name": name},
            }
        )
    for cpu in cpus:
        events.append(
            {
                "name": "thread_name",
                "ph": "M",
                "pid": CPU_PID,
                "tid": cpu,
                "args": {"name": f"Cpu {cpu:03d}"},
            }
        )
    return events


def build_instant_trace(
    samples: list[dict],
    thread_names: dict[int, str],
    cpus: list[int],
) -> dict:
    """Each sample becomes a vertical instant marker on its swimlane."""
    events: list[dict] = _emit_metadata(thread_names, cpus)

    for s in samples:
        leaf = s["frames"][0] if s["frames"] else "[unknown]"
        label = _truncate(leaf)
        args = {
            "cpu": s["cpu"],
            "tid": s["tid"],
            "comm": s["comm"],
            "stack": s["frames"],
        }
        for pid, lane_tid in ((THREAD_PID, s["tid"]), (CPU_PID, s["cpu"])):
            events.append(
                {
                    "name": label,
                    "cat": "sample",
                    "ph": "I",
                    "ts": s["time_us"],
                    "pid": pid,
                    "tid": lane_tid,
                    "s": "t",
                    "args": args,
                }
            )

    return {"traceEvents": events, "displayTimeUnit": "ms"}


def build_flamechart_trace(
    samples: list[dict],
    thread_names: dict[int, str],
    cpus: list[int],
    *,
    max_gap_us: int = 50_000,
) -> dict:
    """Stitch adjacent samples into begin/end pairs along common callstack prefix.

    Algorithm (per thread):
        Group samples by tid, sort by time_us.
        Maintain ``stack_open`` = list of (root-first) frame names currently
        "in progress" (B emitted, E pending).
        For each new sample s:
            new_stack = reversed(s.frames)   # root-first
            common = longest matching prefix between stack_open and new_stack
            If gap to previous sample > max_gap_us:
                close everything (the thread was off-CPU long enough that we
                cannot reasonably claim continuity).
                Emit fresh B for entire new_stack at s.time_us.
            Else:
                Emit E for stack_open[common:] in LIFO order at s.time_us.
                Emit B for new_stack[common:] in caller→callee order at s.time_us.
            stack_open = new_stack.
        After last sample on the thread: emit E for everything remaining at
        last_time_us + 1 us so a non-zero-width final bar exists.

    The same passes are run twice — once for the per-thread track, once for
    the per-CPU track — because Chrome Trace B/E pairs are scoped to (pid, tid).

    ``max_gap_us``: when two adjacent samples on the same thread are farther
    apart than this, treat as discontinuous (the thread likely slept or was
    descheduled). Default 50 ms, well above WBC tick (2 ms) and MPC tick (8 ms).
    """
    events: list[dict] = _emit_metadata(thread_names, cpus)

    def emit_for_lane(grouped: dict[int, list[dict]], pid_for_lane: int, lane_field: str) -> None:
        for lane_id, lane_samples in grouped.items():
            lane_samples.sort(key=lambda s: s["time_us"])
            stack_open: list[str] = []
            prev_ts: int | None = None
            last_ts: int = 0

            for s in lane_samples:
                ts = s["time_us"]
                last_ts = ts
                # root-first ordering for prefix comparison
                new_stack = list(reversed(s["frames"]))

                discontinuous = prev_ts is not None and (ts - prev_ts) > max_gap_us and stack_open

                if discontinuous:
                    # Close everything from the previous interval at prev_ts + 1
                    close_ts = prev_ts + 1 if prev_ts is not None else ts
                    for _ in range(len(stack_open)):
                        events.append(
                            {
                                "ph": "E",
                                "ts": close_ts,
                                "pid": pid_for_lane,
                                "tid": lane_id,
                                "cat": "frame",
                            }
                        )
                    stack_open = []

                # Common prefix
                common = 0
                limit = min(len(stack_open), len(new_stack))
                while common < limit and stack_open[common] == new_stack[common]:
                    common += 1

                # Close stale frames in LIFO order at ts
                for _ in range(len(stack_open) - common):
                    events.append(
                        {
                            "ph": "E",
                            "ts": ts,
                            "pid": pid_for_lane,
                            "tid": lane_id,
                            "cat": "frame",
                        }
                    )

                # Open new frames in caller → callee order at ts
                for frame in new_stack[common:]:
                    events.append(
                        {
                            "name": _truncate(frame),
                            "cat": "frame",
                            "ph": "B",
                            "ts": ts,
                            "pid": pid_for_lane,
                            "tid": lane_id,
                            "args": {
                                "full_name": frame,
                                "cpu": s["cpu"],
                                "comm": s["comm"],
                            },
                        }
                    )

                stack_open = new_stack
                prev_ts = ts

            # Close everything remaining at last_ts + 1 us
            close_ts = last_ts + 1
            for _ in range(len(stack_open)):
                events.append(
                    {
                        "ph": "E",
                        "ts": close_ts,
                        "pid": pid_for_lane,
                        "tid": lane_id,
                        "cat": "frame",
                    }
                )

    # Per-thread lane
    by_tid: dict[int, list[dict]] = defaultdict(list)
    for s in samples:
        by_tid[s["tid"]].append(s)
    emit_for_lane(by_tid, THREAD_PID, "tid")

    # Per-CPU lane (independent stitch — the same "function" can show up on
    # different CPUs if a thread migrates, so we treat each CPU lane separately)
    by_cpu: dict[int, list[dict]] = defaultdict(list)
    for s in samples:
        by_cpu[s["cpu"]].append(s)
    emit_for_lane(by_cpu, CPU_PID, "cpu")

    return {"traceEvents": events, "displayTimeUnit": "ms"}


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n\n")[0])
    p.add_argument(
        "--input",
        "-i",
        type=Path,
        help="perf.data file (will be passed to `perf script -i`). Mutually exclusive "
        "with stdin: omit to read perf script output from stdin.",
    )
    p.add_argument(
        "--output", "-o", type=Path, required=True, help="Output Chrome Trace JSON path."
    )
    p.add_argument(
        "--max-depth",
        type=int,
        default=5,
        help="Maximum callstack frames per sample to keep, leaf-first (default: 5). "
        "Use 0 for no limit; smaller numbers shrink the JSON and speed Perfetto load.",
    )
    p.add_argument(
        "--mode",
        choices=("flamechart", "instant"),
        default="flamechart",
        help="flamechart (default): horizontal bars labelled with function names, "
        "duration inferred from common-prefix continuity between adjacent samples. "
        "instant: vertical markers at exact sample timestamps.",
    )
    p.add_argument(
        "--max-gap-us",
        type=int,
        default=50_000,
        help="Flamechart mode only: when two adjacent samples on the same lane are "
        "farther apart than this many microseconds, treat as discontinuous and "
        "close all open frames (default 50000 = 50 ms).",
    )
    args = p.parse_args(argv)

    if args.input is not None:
        if not args.input.is_file():
            print(f"[perf_to_chrome] input not found: {args.input}", file=sys.stderr)
            return 1
        proc = subprocess.Popen(
            ["perf", "script", "-i", str(args.input)],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
            text=True,
        )
        assert proc.stdout is not None
        samples, thread_names, cpus = parse_perf_script(proc.stdout, max_depth=args.max_depth)
        proc.wait()
        if proc.returncode != 0:
            print(f"[perf_to_chrome] perf script exited {proc.returncode}", file=sys.stderr)
            return proc.returncode
    else:
        samples, thread_names, cpus = parse_perf_script(sys.stdin, max_depth=args.max_depth)

    if not samples:
        print(
            "[perf_to_chrome] no samples parsed — input empty or format unrecognised",
            file=sys.stderr,
        )
        return 1

    if args.mode == "flamechart":
        trace = build_flamechart_trace(samples, thread_names, cpus, max_gap_us=args.max_gap_us)
    else:
        trace = build_instant_trace(samples, thread_names, cpus)

    args.output.parent.mkdir(parents=True, exist_ok=True)
    with args.output.open("w") as f:
        json.dump(trace, f)

    per_tid: dict[int, int] = defaultdict(int)
    for s in samples:
        per_tid[s["tid"]] += 1
    top_tids = sorted(per_tid.items(), key=lambda kv: -kv[1])[:5]
    print(
        f"[perf_to_chrome] mode={args.mode}  wrote {args.output} "
        f"({len(samples)} samples, {len(thread_names)} threads, {len(cpus)} CPUs, "
        f"{len(trace['traceEvents'])} events)",
        file=sys.stderr,
    )
    for tid, count in top_tids:
        print(f"[perf_to_chrome]   {thread_names[tid]:<30s} {count:6d} samples", file=sys.stderr)
    print(
        "[perf_to_chrome] open https://ui.perfetto.dev and drag-drop the JSON",
        file=sys.stderr,
    )
    return 0


if __name__ == "__main__":
    sys.exit(main())
