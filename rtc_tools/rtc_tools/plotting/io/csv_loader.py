"""CSV loading + numeric coercion.

Legacy CSVs with truncated headers (sensor_log files written before the
inference columns were appended to the writer) are no longer supported. They
are rejected with a clear error so silent column padding cannot mask
regressions. Re-record sessions with the current code.
"""

import csv
import re


class LegacyCsvError(ValueError):
    """Raised when a CSV's header column count does not match its data rows."""


def _check_header_matches_data(filepath: str) -> None:
    """Reject CSVs whose data rows are wider than the header.

    Reads the header + up to 10 data rows. If any data row has more columns
    than the header, raise LegacyCsvError. This catches the legacy sensor_log
    case (header < data) without doing column padding.
    """
    with open(filepath) as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            return  # empty file: let pandas raise EmptyDataError downstream
        header_count = len(header)
        for _, row in zip(range(10), reader, strict=False):
            if len(row) > header_count:
                raise LegacyCsvError(
                    f"Legacy CSV without complete header: {filepath} "
                    f"(header has {header_count} columns, data row has "
                    f"{len(row)}). Re-record this session with the current "
                    f"code; legacy header repair is no longer supported."
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


def load_log_csv(filepath, log_type):
    """Load a CSV by log_type.

    Returns the DataFrame. Raises:
      - pd.errors.EmptyDataError on empty input.
      - LegacyCsvError if header column count is shorter than data rows.
    Caller handles ParserError fallback if desired.
    """
    import pandas as pd

    _check_header_matches_data(filepath)
    df = pd.read_csv(filepath)
    # Before coercion: the stamped names must be normalized while the frame
    # still has them, and the bare names are what _STR_COLS is keyed on.
    _normalize_mask_columns(df)
    _coerce_numeric_columns(df)
    _ensure_timestamp_column(df)
    return df
