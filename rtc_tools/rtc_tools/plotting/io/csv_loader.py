"""CSV loading + numeric coercion.

Legacy CSVs with truncated headers (sensor_log files written before the
inference columns were appended to the writer) are no longer supported. They
are rejected with a clear error so silent column padding cannot mask
regressions. Re-record sessions with the current code.
"""

import csv

import pandas as pd


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
_STR_COLS = {"goal_type", "command_type", "timestamp"}


def _coerce_numeric_columns(df):
    """Convert object-dtype columns to numeric (NaN on failure).

    timestamp + known enum/string columns are excluded.
    Mutates df in place; returns df for chaining.
    """
    for col in df.columns:
        if col in _STR_COLS:
            continue
        if df[col].dtype == object:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    return df


def load_log_csv(filepath, log_type):
    """Load a CSV by log_type.

    Returns the DataFrame. Raises:
      - pd.errors.EmptyDataError on empty input.
      - LegacyCsvError if header column count is shorter than data rows.
    Caller handles ParserError fallback if desired.
    """
    _check_header_matches_data(filepath)
    df = pd.read_csv(filepath)
    _coerce_numeric_columns(df)
    return df
