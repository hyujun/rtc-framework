"""CSV loading + numeric coercion + run-boundary selection.

Legacy CSVs with truncated headers (sensor_log files written before the
inference columns were appended to the writer) are no longer supported. They
are rejected with a clear error so silent column padding cannot mask
regressions. Re-record sessions with the current code.

Timing CSVs additionally carry a `run_id` column (#376). Session directories
are minute-resolution and the timing logger appends, so two starts within one
minute land in the same file; loading such a file whole makes every `n / span`
rate 17%-ish wrong with no other symptom. `select_run` keeps the LAST run and
says so loudly. Files without the column (archived sessions) load as one run.
"""

import csv
import re


class LegacyCsvError(ValueError):
    """Raised when a CSV's header column count does not match its data rows."""


class UnknownRunIdError(ValueError):
    """Raised when an explicitly requested run_id is not present in the CSV."""


def _check_header_matches_data(filepath: str) -> None:
    """Reject CSVs whose data rows are wider than the header; warn if narrower.

    Reads the header + up to 10 data rows. If any data row has more columns
    than the header, raise LegacyCsvError. This catches the legacy sensor_log
    case (header < data) without doing column padding.

    A row NARROWER than the header is a different animal and is not an error:
    the DeviceSensorLog writer sizes its header from the names captured at
    configure time but each row from the fingertip count the device actually
    reported that tick, so a device that reported none legitimately emits a
    two-field row (see test_device_log_pods.cpp RowRespectsRuntimeNumFingertips).
    pandas NaN-fills the difference silently, which downstream is
    indistinguishable from a sensor that recorded nothing but zeros — so warn,
    because "this session has no fingertip data" is almost never what the
    person plotting it expects to find.
    """
    with open(filepath) as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            return  # empty file: let pandas raise EmptyDataError downstream
        header_count = len(header)
        narrow = 0
        narrowest = header_count
        for _, row in zip(range(10), reader, strict=False):
            if len(row) > header_count:
                raise LegacyCsvError(
                    f"Legacy CSV without complete header: {filepath} "
                    f"(header has {header_count} columns, data row has "
                    f"{len(row)}). Re-record this session with the current "
                    f"code; legacy header repair is no longer supported."
                )
            if len(row) < header_count:
                narrow += 1
                narrowest = min(narrowest, len(row))
        if narrow:
            print(
                f"  WARNING: {filepath} has data rows narrower than its header "
                f"({narrowest} vs {header_count} columns in the first {narrow} of 10 "
                f"sampled rows). The missing columns load as NaN — the producing "
                f"device reported fewer channels than were declared at configure time."
            )


# Columns whose values are intentionally string/categorical — never coerce.
# `phase` is the WbcDiagLog FSM-phase name (idle/approach/.../fallback).
_STR_COLS = {"goal_type", "command_type", "phase", "timestamp"}

# Bitmask columns stamp their bit order into the column name, e.g.
# `contact_mask[thumb|index|middle]` (#234 P-14 — the contact→role mapping used
# to live only in that run's ROS log). Plotters want a stable `contact_mask`
# key, so the suffix is stripped here and the decoded order is published on
# `df.attrs["mask_roles"]` instead. This is the same normalization boundary
# `_ensure_timestamp_column` uses, for the same reason: plotters stay
# schema-agnostic.
_MASK_SUFFIX_RE = re.compile(r"^(?P<name>[A-Za-z0-9_]+)\[(?P<roles>[^\]]*)\]$")


def _normalize_mask_columns(df):
    """Strip `[role|role|...]` stamps from column names into ``df.attrs``.

    Adds ``df.attrs["mask_roles"]`` mapping the bare column name to the list of
    role names, bit k = ``roles[k]``. Columns without a stamp are untouched and
    contribute no entry, so a legacy CSV simply yields an empty mapping.

    Mutates df in place; returns df for chaining.
    """
    renames = {}
    mask_roles = {}
    for col in df.columns:
        m = _MASK_SUFFIX_RE.match(col)
        if m is None:
            continue
        name = m.group("name")
        # A stamped and an unstamped column of the same name in one file would
        # collide on rename; keep the original rather than silently dropping a
        # column, since that means the producer emitted something unexpected.
        if name in df.columns:
            continue
        renames[col] = name
        roles = m.group("roles")
        mask_roles[name] = roles.split("|") if roles else []
    if renames:
        df.rename(columns=renames, inplace=True)
    df.attrs["mask_roles"] = mask_roles
    return df


def _coerce_numeric_columns(df):
    """Convert object-dtype columns to numeric (NaN on failure).

    timestamp + known enum/string columns are excluded.
    Mutates df in place; returns df for chaining.
    """
    import pandas as pd

    for col in df.columns:
        if col in _STR_COLS:
            continue
        if df[col].dtype == object:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    return df


def _ensure_timestamp_column(df):
    """Derive a `timestamp` column (seconds since first sample) when absent.

    Producer side writes one of:
      - `t_wall_ns`     (uint64 ns since boot — timing CSVs)
      - `t_relative_s`  (float seconds since session start — Phase C state/sensor)
    Plotters universally read `df["timestamp"]`. This is the single boundary
    where unit normalization happens, so plotters stay schema-agnostic.

    No-op if `timestamp` already exists or if neither source column is present.
    Mutates df in place; returns df for chaining.
    """
    if "timestamp" in df.columns:
        return df
    if "t_wall_ns" in df.columns:
        t0 = df["t_wall_ns"].iloc[0]
        df["timestamp"] = (df["t_wall_ns"] - t0) / 1e9
    elif "t_relative_s" in df.columns:
        df["timestamp"] = df["t_relative_s"]
    return df


def select_run(df, filepath, run_id=None):
    """Reduce a multi-run timing CSV to one run; announce what was dropped.

    The `run_id` column (#376) is what a restart inside one minute-resolution
    session directory leaves behind. Merging the runs is never right — the
    interesting failure is not that the plot looks odd but that `n / span`
    silently reports a rate that never happened (#349: 413 Hz for a loop that
    ticked at 500 Hz, with jitter p99 of 0.225 µs insisting the period was
    fine). So: keep the last run, name the ones dropped.

    `run_id=None` selects the last run; an explicit value selects that run and
    raises UnknownRunIdError when it is absent (a typo must not silently plot
    a different run than the one asked for).

    Publishes ``df.attrs["runs"]`` — the (run_id, row count) pairs seen, in
    file order — and ``df.attrs["run_id"]`` for the one selected, so callers
    can report on runs they did not get. No-op for files without the column:
    they predate #376 and are one run by definition.

    Returns the selected frame, which is `df` itself when the file holds a
    single run. Row data is never modified; only ``attrs`` is written.
    """
    if "run_id" not in df.columns or len(df) == 0:
        return df

    counts = df["run_id"].value_counts(sort=False, dropna=False)
    runs = [(rid, int(n)) for rid, n in counts.items()]
    df.attrs["runs"] = runs

    if run_id is None:
        if len(runs) <= 1:
            df.attrs["run_id"] = runs[0][0]
            return df
        selected = df["run_id"].iloc[-1]
        dropped = ", ".join(f"run {rid} ({n} rows)" for rid, n in runs if rid != selected)
        print(
            f"  WARNING: {filepath} holds {len(runs)} runs — the controller was "
            f"restarted inside one session directory. Plotting the LAST run "
            f"(run {selected}, {int(counts[selected])} rows) and dropping: {dropped}. "
            f"Pass --run-id to pick another; rates computed over the whole file "
            f"would be meaningless."
        )
    else:
        selected = type(df["run_id"].iloc[0])(run_id)
        if selected not in set(df["run_id"]):
            available = ", ".join(str(rid) for rid, _ in runs)
            raise UnknownRunIdError(
                f"run_id {run_id} is not in {filepath} (available: {available})"
            )

    out = df[df["run_id"] == selected].reset_index(drop=True)
    out.attrs.update(df.attrs)
    out.attrs["run_id"] = selected
    return out


def load_log_csv(filepath, log_type, run_id=None):
    """Load a CSV by log_type.

    `run_id` picks one run out of a timing CSV that holds several (#376);
    the default keeps the last one. See `select_run`.

    Returns the DataFrame. Raises:
      - pd.errors.EmptyDataError on empty input.
      - LegacyCsvError if header column count is shorter than data rows.
      - UnknownRunIdError if an explicit run_id is not in the file.
    Caller handles ParserError fallback if desired.
    """
    import pandas as pd

    _check_header_matches_data(filepath)
    df = pd.read_csv(filepath)
    # Before coercion: the stamped names must be normalized while the frame
    # still has them, and the bare names are what _STR_COLS is keyed on.
    _normalize_mask_columns(df)
    _coerce_numeric_columns(df)
    # Run selection precedes timestamp derivation on purpose: `timestamp` is
    # measured from the first row's t_wall_ns, and that origin must be the
    # start of the run being plotted, not of the run that was discarded.
    df = select_run(df, filepath, run_id)
    _ensure_timestamp_column(df)
    return df
