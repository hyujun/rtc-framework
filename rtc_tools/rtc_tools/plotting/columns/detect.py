"""Column detection helpers — variable-DOF, named-or-numeric, with caching.

`_column_cache` is keyed by `(id(df), prefix, count)`. Always call
`invalidate_column_cache()` after loading a new DataFrame.
"""

_column_cache = {}

# Categorical columns that share a joint-channel prefix but are NOT per-joint
# numeric series. The state_log writer emits `command_<joint>` columns followed
# by a single categorical `command_type` (and `goal_type`); a naive
# `startswith("command_")` would otherwise treat `command_type` as an extra
# joint channel. The remainder after the prefix (e.g. "type") identifies these.
_NON_JOINT_REMAINDERS = frozenset({"type"})


def _cache_key(df_id, prefix, count):
    return (df_id, prefix, count)


def invalidate_column_cache():
    """캐시 초기화 (새 DataFrame 로드 시)."""
    _column_cache.clear()


def _joint_match_cols(df, prefix):
    """prefix로 시작하는 per-joint 컬럼 (categorical 컬럼 제외)."""
    return [
        c
        for c in df.columns
        if c.startswith(prefix) and c[len(prefix) :] not in _NON_JOINT_REMAINDERS
    ]


def detect_num_channels(df, prefix):
    """Auto-detect number of channels from CSV headers matching prefix."""
    return len(_joint_match_cols(df, prefix))


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

    named_cols = _joint_match_cols(df, prefix)
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
    named = _joint_match_cols(df, prefix)
    if len(named) >= count:
        return True
    return all(f"{prefix}{i}" in df.columns for i in range(count))


# Phase C sensor POD packs kSensorValuesPerFingertip=11 values per fingertip
# as `<label>_raw_<v>` / `<label>_filt_<v>` (the writer doesn't know the value
# semantics). By documented convention the block is 8 barometer values
# (v 0..7) followed by 3 ToF values (v 8..10). Legacy logs instead used
# `baro[_raw]_<label>_<b>` / `tof[_raw]_<label>_<t>`. The resolvers below accept
# both schemas so plotters stay schema-agnostic.
BARO_VALUE_COUNT = 8  # block values 0..7
TOF_VALUE_COUNT = 3  # block values 8..10


def detect_fingertip_labels(df):
    """filtered fingertip 라벨 목록을 자동 감지.

    Phase C `<label>_filt_0` 우선, 없으면 legacy `baro_<label>_0` (baro_raw_ 제외).
    """
    labels = []
    for col in df.columns:  # Phase C: <label>_filt_<v>
        if col.endswith("_filt_0"):
            middle = col[: -len("_filt_0")]
            if f"{middle}_filt_1" in df.columns and middle not in labels:
                labels.append(middle)
    if labels:
        return labels
    for col in df.columns:  # legacy: baro_<label>_<b>
        if col.startswith("baro_") and not col.startswith("baro_raw_") and col.endswith("_0"):
            middle = col[len("baro_") : -len("_0")]
            if f"baro_{middle}_1" in df.columns and middle not in labels:
                labels.append(middle)
    return labels


def detect_fingertip_labels_raw(df):
    """raw fingertip 라벨 목록을 자동 감지.

    Phase C `<label>_raw_0` 우선, 없으면 legacy `baro_raw_<label>_0`.
    """
    labels = []
    for col in df.columns:  # Phase C: <label>_raw_<v>
        if col.endswith("_raw_0"):
            middle = col[: -len("_raw_0")]
            if f"{middle}_raw_1" in df.columns and middle not in labels:
                labels.append(middle)
    if labels:
        return labels
    for col in df.columns:  # legacy: baro_raw_<label>_<b>
        if col.startswith("baro_raw_") and col.endswith("_0"):
            middle = col[len("baro_raw_") : -len("_0")]
            if f"baro_raw_{middle}_1" in df.columns and middle not in labels:
                labels.append(middle)
    return labels


def baro_col(df, label, b, raw=False):
    """Resolve barometer channel `b` (0..7) column for `label`, or None.

    Phase C `<label>_raw_/_filt_<b>` 우선, 없으면 legacy `baro[_raw]_<label>_<b>`.
    """
    if not 0 <= b < BARO_VALUE_COUNT:  # guard: block values 8..10 are ToF, not baro
        return None
    phase_c = f"{label}_{'raw' if raw else 'filt'}_{b}"
    if phase_c in df.columns:
        return phase_c
    legacy = f"baro_raw_{label}_{b}" if raw else f"baro_{label}_{b}"
    return legacy if legacy in df.columns else None


def tof_col(df, label, t, raw=False):
    """Resolve ToF channel `t` (0..2) column for `label`, or None.

    Phase C ToF occupies block values 8..10, so `<label>_*_<8+t>`; legacy
    `tof[_raw]_<label>_<t>`.
    """
    if not 0 <= t < TOF_VALUE_COUNT:
        return None
    phase_c = f"{label}_{'raw' if raw else 'filt'}_{BARO_VALUE_COUNT + t}"
    if phase_c in df.columns:
        return phase_c
    legacy = f"tof_raw_{label}_{t}" if raw else f"tof_{label}_{t}"
    return legacy if legacy in df.columns else None


FORCE_AXES = ("fx", "fy", "fz")


def ft_force_col(df, label, axis, filt=False):
    """Resolve the raw or LPF'd force column for `label` / `axis`, or None.

    `axis` is one of FORCE_AXES. The filtered columns are suffixed
    (`ft_<label>_fx_filt`) rather than prefixed, so the raw-force detectors —
    which match on the trailing `_fx` token — cannot mistake them for a second
    set of raw columns.
    """
    if axis not in FORCE_AXES:
        return None
    col = f"ft_{label}_{axis}_filt" if filt else f"ft_{label}_{axis}"
    return col if col in df.columns else None


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
