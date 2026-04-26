"""CSV loading + sensor_log header repair + numeric coercion.

Legacy sensor_log CSVs may have header < data column count when inference
columns were added without bumping the header writer. We detect the gap and
reconstruct the missing `ft_{label}_{comp}` headers.
"""

import csv

import pandas as pd

# DataLogger constants — mirror urtc::kSensorValuesPerFingertip etc.
_BARO_COUNT = 8
_TOF_COUNT = 3
_SENSORS_PER_FT = _BARO_COUNT + _TOF_COUNT  # 11
_FT_COMPS = ["contact", "fx", "fy", "fz", "ux", "uy", "uz"]
_FT_VALUES_PER_FT = len(_FT_COMPS)  # 7
_VALUES_PER_FT = 2 * _SENSORS_PER_FT + _FT_VALUES_PER_FT  # 29


def _detect_csv_column_mismatch(filepath):
    """CSV 헤더와 데이터 행의 컬럼 수 차이를 감지.

    Returns (header_count, data_count, header_line).
    두 값이 같으면 불일치 없음.
    """
    with open(filepath, "r") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            return (0, 0, [])
        header_count = len(header)
        data_count = header_count
        for _, row in zip(range(10), reader):
            if len(row) > data_count:
                data_count = len(row)
    return (header_count, data_count, header)


def _rebuild_sensor_log_header(original_header, data_col_count):
    """누락된 inference 컬럼을 재구성하여 data_col_count에 맞춤."""
    missing = data_col_count - len(original_header)
    if missing <= 0:
        return original_header

    ft_labels = []
    for col in original_header:
        if col.startswith("baro_raw_") and col.endswith("_0"):
            label = col[len("baro_raw_") : -len("_0")]
            ft_labels.append(label)

    inference_cols = []
    if ft_labels:
        for label in ft_labels:
            for comp in _FT_COMPS:
                inference_cols.append(f"ft_{label}_{comp}")
    else:
        inference_cols = [f"inference_{i}" for i in range(missing)]

    inference_cols = inference_cols[:missing]
    while len(inference_cols) < missing:
        inference_cols.append(f"inference_{len(inference_cols)}")

    return original_header + inference_cols


def _load_sensor_log_csv(filepath):
    """sensor_log CSV를 로드. 컬럼 수 불일치 시 자동 복구.

    Returns (df, repaired: bool).
    """
    header_count, data_count, original_header = _detect_csv_column_mismatch(filepath)

    if header_count == 0:
        raise pd.errors.EmptyDataError(f"Empty CSV: {filepath}")

    if header_count >= data_count:
        df = pd.read_csv(filepath)
        return df, False

    print(
        f"  Detected column mismatch: header has {header_count} cols, "
        f"data has {data_count} cols (missing {data_count - header_count} inference cols)"
    )
    new_header = _rebuild_sensor_log_header(original_header, data_count)
    print(
        f"  Reconstructed header: added {len(new_header) - header_count} inference columns"
    )

    df = pd.read_csv(
        filepath, header=0, names=new_header, skiprows=1, on_bad_lines="warn"
    )
    return df, True


# Columns whose values are intentionally string/categorical — never coerce.
_STR_COLS = {"goal_type", "command_type", "timestamp"}


def _coerce_numeric_columns(df):
    """레거시 CSV에서 컬럼 수 불일치로 문자열이 밀려 들어간 경우 숫자로 변환.

    timestamp + 알려진 enum/string 컬럼은 제외.
    Mutates df in place; returns df for chaining.
    """
    for col in df.columns:
        if col in _STR_COLS:
            continue
        if df[col].dtype == object:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    return df


def load_log_csv(filepath, log_type):
    """Load a CSV by log_type, with sensor_log header repair where needed.

    Returns the DataFrame. Raises pd.errors.EmptyDataError on empty input.
    Caller handles ParserError fallback if desired.
    """
    if log_type in ("sensor_log", "device"):
        df, repaired = _load_sensor_log_csv(filepath)
        if repaired:
            print(f"  Loaded {len(df)} rows (header repaired).")
    else:
        df = pd.read_csv(filepath)

    _coerce_numeric_columns(df)
    return df
