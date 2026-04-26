"""Column detection helpers — variable-DOF, named-or-numeric, with caching.

`_column_cache` is keyed by `(id(df), prefix, count)`. Always call
`invalidate_column_cache()` after loading a new DataFrame.
"""

_column_cache = {}


def _cache_key(df_id, prefix, count):
    return (df_id, prefix, count)


def invalidate_column_cache():
    """캐시 초기화 (새 DataFrame 로드 시)."""
    _column_cache.clear()


def detect_num_channels(df, prefix):
    """Auto-detect number of channels from CSV headers matching prefix."""
    return len([c for c in df.columns if c.startswith(prefix)])


def detect_joint_columns(df, prefix, count=None):
    """CSV 헤더에서 prefix로 시작하는 컬럼을 감지하여 (column_names, display_names) 반환.

    count=None이면 CSV에서 자동 감지.
    Named headers: 'goal_pos_shoulder_pan_joint' 형태
    Numeric headers: 'goal_pos_0' 형태
    둘 다 자동 감지. 결과는 캐싱됨.
    """
    if count is None:
        count = detect_num_channels(df, prefix)
    if count == 0:
        return ([], [])
    key = _cache_key(id(df), prefix, count)
    if key in _column_cache:
        return _column_cache[key]

    named_cols = [c for c in df.columns if c.startswith(prefix)]
    if len(named_cols) >= count:
        display = [c[len(prefix) :] for c in named_cols[:count]]
        result = (named_cols[:count], display)
        _column_cache[key] = result
        return result

    numeric_cols = [f"{prefix}{i}" for i in range(count)]
    if all(c in df.columns for c in numeric_cols):
        result = (numeric_cols, [f"J{i}" for i in range(count)])
        _column_cache[key] = result
        return result

    result = ([], [])
    _column_cache[key] = result
    return result


def has_columns(df, prefix, count):
    """Check if df has columns like '{prefix}0' .. '{prefix}{count-1}'.
    Also supports named columns.
    """
    named = [c for c in df.columns if c.startswith(prefix)]
    if len(named) >= count:
        return True
    return all(f"{prefix}{i}" in df.columns for i in range(count))


def detect_fingertip_labels(df):
    """CSV 헤더에서 fingertip 라벨 목록을 자동 감지.

    baro_*_0 패턴의 컬럼을 찾아 중간 라벨을 추출 (baro_raw_* 제외).
    """
    labels = []
    for col in df.columns:
        if (
            col.startswith("baro_")
            and not col.startswith("baro_raw_")
            and col.endswith("_0")
        ):
            middle = col[len("baro_") : -len("_0")]
            if f"baro_{middle}_1" in df.columns:
                labels.append(middle)
    return labels


def detect_fingertip_labels_raw(df):
    """raw barometer 컬럼에서 fingertip 라벨 목록을 자동 감지."""
    labels = []
    for col in df.columns:
        if col.startswith("baro_raw_") and col.endswith("_0"):
            middle = col[len("baro_raw_") : -len("_0")]
            if f"baro_raw_{middle}_1" in df.columns:
                labels.append(middle)
    return labels


def detect_ft_labels(df):
    """FT inference output fingertip 라벨 목록을 자동 감지.

    Prefer 3-head model (`ft_{label}_contact`); fall back to legacy 6-output
    (`ft_{label}_fx` + `ft_{label}_fy`).
    """
    labels = []
    for col in df.columns:
        if col.startswith("ft_") and col.endswith("_contact"):
            middle = col[len("ft_") : -len("_contact")]
            if middle != "valid" and f"ft_{middle}_fx" in df.columns:
                labels.append(middle)
    if not labels:
        for col in df.columns:
            if col.startswith("ft_") and col.endswith("_fx"):
                middle = col[len("ft_") : -len("_fx")]
                if middle != "valid" and f"ft_{middle}_fy" in df.columns:
                    labels.append(middle)
    return labels


def has_motor_columns(df):
    """state_log CSV에 motor_pos_* 컬럼이 존재하는지 확인."""
    return any(c.startswith("motor_pos_") for c in df.columns)
