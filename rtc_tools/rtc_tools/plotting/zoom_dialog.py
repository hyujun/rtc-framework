"""Right-click "zoom to a typed range" dialog for `plot_rtc_log` figures.

Why this exists
---------------
The interactive toolbar only zooms by dragging a rectangle, so a window like
18.93–34.55 s is neither typeable nor reproducible, and matplotlib's built-in
numeric axis editor (``qt_editor/figureoptions.py``, the "Left/Right/Bottom/
Top" dialog) is **Qt-only** while `plot_rtc_log` runs on TkAgg.

Two things this does that a toolbar drag cannot:

* **Apply one x window to every subplot of the figure.** Only 7 of the figures
  this package builds pass ``sharex=``; the multi-joint grids
  (`robot_positions`, `robot_velocities`, `motor_*`, `sensor_*`, `timing_*`)
  build fully independent axes, so a drag zooms one cell out of six. Hence
  `ZoomRequest.x_all` defaults to True.
* **Refit y to the x window.** ``Axes.autoscale()`` scores the whole dataset,
  so narrowing x alone leaves the trace flat inside a full-run y range.

Attachment
----------
`attach()` runs once per open figure from `plot_rtc_log.main()` just before
`plt.show()`. Every figure is still open at that point because
`layout.disable_close()` has neutered `plt.close()`, so no plotter needs to
know this module exists.

The transform guard
-------------------
`data_ylim()` folds in only artists whose transform *is* ``ax.transData``.
Everything else on these figures carries one axis in **axes** coordinates:
``axvline`` (grasp's phase bands) has ydata ``[0, 1]``, ``axhline`` (timing's
budget line, robot's zero line) has xdata ``[0, 1]``, and momentum's
invalid-span shading is a ``fill_between`` over a blended transform with
y spanning 0..1. Folding any of those in pins y to [0, 1] on most figures.
"""

from dataclasses import dataclass, replace
from pathlib import Path

import numpy as np

from rtc_tools.plotting import layout

# matplotlib reports the right button as MouseButton.RIGHT, which compares
# equal to 3; comparing against the int keeps this importable without pulling
# in a pyplot-dependent enum at module scope.
RIGHT_BUTTON = 3

# Figure attribute carrying the PNG stem the plotter used, stamped by
# `pipelines.registry.run_pipeline` so "Save PNG" can name its output after
# the figure it came from rather than after a pyplot figure number.
PNG_STEM_ATTR = "_rtc_png_stem"

# Fraction of the visible span added above and below a refitted y range, so a
# trace at the window's extreme does not sit exactly on the frame.
Y_PAD_FRACTION = 0.05


@dataclass(frozen=True)
class ZoomRequest:
    """What the dialog asks for. `None` limits mean "leave that axis alone"."""

    xlim: tuple[float, float] | None = None
    ylim: tuple[float, float] | None = None
    # True for x by default: most figures here have independent axes, so the
    # useful action is "this window, every subplot" (see module docstring).
    x_all: bool = True
    # False for y by default: a grid can mix units down its rows
    # (`wbc_task_trajectory` is x/y/z in m then roll/pitch/yaw in rad), so a
    # single y range across the figure is not generally meaningful.
    y_all: bool = False
    # Ignore `ylim` and refit y from the data inside the x window instead.
    y_auto: bool = False
    # Restore the limits captured when the dialog was attached; overrides
    # every other field.
    reset: bool = False
    save: bool = False


def initial_request(ax) -> ZoomRequest:
    """Prefill a request from what `ax` currently shows."""
    return ZoomRequest(xlim=tuple(ax.get_xlim()), ylim=tuple(ax.get_ylim()))


# ── y refit ───────────────────────────────────────────────────────────────


def _line_xy(line):
    """(x, y) of a Line2D as float arrays with masked entries as NaN."""
    x = np.ma.filled(np.ma.asarray(line.get_xdata()), np.nan).astype(float, copy=False)
    y = np.ma.filled(np.ma.asarray(line.get_ydata()), np.nan).astype(float, copy=False)
    return x, y


def _window_extremes(x, y, lo, hi):
    """(min, max) of `y` where `x` is inside [lo, hi], or None."""
    if x.size == 0 or x.size != y.size:
        return None
    inside = (x >= lo) & (x <= hi) & np.isfinite(y)
    if not inside.any():
        return None
    seg = y[inside]
    return (float(seg.min()), float(seg.max()))


def data_ylim(axes, lo, hi, pad=Y_PAD_FRACTION):
    """Union y range of everything drawn between x=lo..hi on `axes`.

    Returns None when nothing data-space is visible in the window, which the
    caller must treat as "leave y as it is" — a figure whose only artists are
    a stackplot over a blended transform, or an x window that misses every
    sample, has no y range to offer.
    """
    ymin, ymax = np.inf, -np.inf

    for ax in axes:
        for line in ax.get_lines():
            # See "The transform guard" in the module docstring: axhline /
            # axvline live in blended coordinates and must not be scored.
            if line.get_transform() is not ax.transData:
                continue
            span = _window_extremes(*_line_xy(line), lo, hi)
            if span is not None:
                ymin, ymax = min(ymin, span[0]), max(ymax, span[1])

        for coll in ax.collections:
            if coll.get_transform() is not ax.transData:
                continue
            for path in coll.get_paths():
                verts = path.vertices
                if verts.size == 0:
                    continue
                span = _window_extremes(
                    verts[:, 0].astype(float, copy=False),
                    verts[:, 1].astype(float, copy=False),
                    lo,
                    hi,
                )
                if span is not None:
                    ymin, ymax = min(ymin, span[0]), max(ymax, span[1])

    if not (np.isfinite(ymin) and np.isfinite(ymax)):
        return None

    span = ymax - ymin
    if span <= 0:
        # A constant channel (a pinned `valid` flag, a zeroed lane) would
        # otherwise collapse to a zero-height axis.
        margin = abs(ymin) * pad if ymin else 0.5
        return (ymin - margin, ymax + margin)
    return (ymin - span * pad, ymax + span * pad)


def y_group(ax):
    """`ax` plus every axes sharing its y axis.

    `sensors.plot_fingertip_force_only_auto` builds `sharey="col"`, so
    refitting one panel in isolation and calling `set_ylim` would propagate
    the last computed range over the whole column and clip the others. Scoring
    the group as a union makes the result independent of iteration order.
    """
    try:
        siblings = list(ax.get_shared_y_axes().get_siblings(ax))
    except Exception:
        siblings = []
    return siblings or [ax]


def autoscale_y_to_xlim(ax, lo, hi):
    """y range that fits everything `ax`'s shared-y group shows in [lo, hi]."""
    return data_ylim(y_group(ax), lo, hi)


# ── applying a request ────────────────────────────────────────────────────


def zoom_png_path(save_dir, stem, lo, hi):
    """Path for a zoomed re-save: `<stem>_zoom_<lo>-<hi>.png`.

    Deliberately never the plain `<stem>.png` the pipeline already wrote:
    that file is the session's full-run record, and the figure on screen has
    been narrowed to a window the person chose interactively.
    """
    return Path(save_dir) / f"{stem}_zoom_{lo:.6g}-{hi:.6g}.png"


def apply_request(fig, ax, req, save_dir=None, home=None):
    """Apply `req` to `fig`, returning the saved path or None.

    `home` is the {axes: (xlim, ylim)} snapshot taken at attach time; it is
    what `req.reset` restores. Restoring the snapshot rather than calling
    `Axes.autoscale()` matters because `relim()` ignores collections, so a
    stackplot figure would not come back to where it started.
    """
    if req.reset:
        for target, (xlim, ylim) in (home or {}).items():
            target.set_xlim(*xlim)
            target.set_ylim(*ylim)
        fig.canvas.draw_idle()
        return None

    x_targets = list(fig.axes) if req.x_all else [ax]
    y_targets = list(fig.axes) if req.y_all else [ax]

    if req.xlim is not None:
        for target in x_targets:
            target.set_xlim(*req.xlim)

    if req.y_auto:
        window = req.xlim if req.xlim is not None else tuple(ax.get_xlim())
        for target in y_targets:
            fitted = autoscale_y_to_xlim(target, *window)
            if fitted is not None:
                target.set_ylim(*fitted)
    elif req.ylim is not None:
        for target in y_targets:
            target.set_ylim(*req.ylim)

    fig.canvas.draw_idle()

    if not req.save:
        return None
    return _save_zoomed(fig, ax, save_dir)


def _save_zoomed(fig, ax, save_dir):
    if not save_dir:
        print(
            "  zoom dialog: no save directory for this session — "
            "use the toolbar's save button instead"
        )
        return None
    stem = getattr(fig, PNG_STEM_ATTR, None) or f"figure{fig.number}"
    lo, hi = ax.get_xlim()
    path = zoom_png_path(save_dir, stem, lo, hi)
    fig.savefig(path, dpi=300, bbox_inches="tight")
    print(f"Saved: {path}")
    return path


# ── request construction from dialog text ─────────────────────────────────

FIELDS = ("x min", "x max", "y min", "y max")


def field_defaults(ax):
    """Prefill strings for `FIELDS`, in that order."""
    (x0, x1), (y0, y1) = ax.get_xlim(), ax.get_ylim()
    return {name: f"{val:.6g}" for name, val in zip(FIELDS, (x0, x1, y0, y1), strict=True)}


def _pair(values, lo_key, hi_key, label):
    """Parse one (min, max) pair; blank-either-side means "no change"."""
    lo_raw, hi_raw = values.get(lo_key, "").strip(), values.get(hi_key, "").strip()
    if not lo_raw or not hi_raw:
        return None
    try:
        lo, hi = float(lo_raw), float(hi_raw)
    except ValueError as exc:
        raise ValueError(f"{label}: not a number ({exc})") from exc
    if not (np.isfinite(lo) and np.isfinite(hi)):
        raise ValueError(f"{label}: must be finite")
    if lo >= hi:
        raise ValueError(f"{label}: min must be below max (got {lo:g} >= {hi:g})")
    return (lo, hi)


def build_request(values, *, x_all=True, y_all=False, y_auto=False, save=False):
    """Turn dialog strings into a ZoomRequest. Raises ValueError on bad input."""
    xlim = _pair(values, "x min", "x max", "x range")
    ylim = None if y_auto else _pair(values, "y min", "y max", "y range")
    return ZoomRequest(xlim=xlim, ylim=ylim, x_all=x_all, y_all=y_all, y_auto=y_auto, save=save)


# ── dialog backends ───────────────────────────────────────────────────────
#
# Tk is what `plot_rtc_log` actually runs on (TkAgg is matplotlib's default
# here), Qt covers MPLBACKEND=QtAgg, and the matplotlib-widgets dialog is the
# last resort that needs no toolkit at all. Each returns True when it put a
# dialog on screen and False when its backend does not apply, so `open_dialog`
# can fall through.


def _title(ax, fig, x_only):
    if x_only:
        return "Zoom — x range (whole figure)"
    try:
        idx = list(fig.axes).index(ax)
    except ValueError:
        idx = 0
    return f"Zoom — subplot #{idx}"


def _open_tk(fig, ax, on_apply, x_only=False):
    canvas = fig.canvas
    if not hasattr(canvas, "get_tk_widget"):
        return False

    import tkinter as tk
    from tkinter import ttk

    parent = canvas.get_tk_widget().winfo_toplevel()
    dlg = tk.Toplevel(parent)
    dlg.title(_title(ax, fig, x_only))
    dlg.transient(parent)
    dlg.resizable(False, False)

    body = ttk.Frame(dlg, padding=10)
    body.grid(row=0, column=0, sticky="nsew")

    defaults = field_defaults(ax)
    entries = {}
    names = FIELDS[:2] if x_only else FIELDS
    for row, name in enumerate(names):
        ttk.Label(body, text=name).grid(row=row, column=0, sticky="e", padx=(0, 6), pady=2)
        entry = ttk.Entry(body, width=14, justify="center")
        entry.insert(0, defaults[name])
        entry.grid(row=row, column=1, pady=2)
        entries[name] = entry

    x_all = tk.BooleanVar(value=True)
    y_all = tk.BooleanVar(value=False)
    y_auto = tk.BooleanVar(value=False)
    save = tk.BooleanVar(value=False)

    opts = ttk.Frame(body)
    opts.grid(row=len(names), column=0, columnspan=2, sticky="w", pady=(8, 0))
    ttk.Checkbutton(opts, text="apply x to every subplot", variable=x_all).grid(
        row=0, column=0, sticky="w"
    )
    if not x_only:
        ttk.Checkbutton(opts, text="apply y to every subplot", variable=y_all).grid(
            row=1, column=0, sticky="w"
        )
        ttk.Checkbutton(opts, text="y: fit to the x window", variable=y_auto).grid(
            row=2, column=0, sticky="w"
        )
    ttk.Checkbutton(opts, text="also save a zoomed PNG", variable=save).grid(
        row=3, column=0, sticky="w"
    )

    status = ttk.Label(body, text="", foreground="#c00")
    status.grid(row=len(names) + 1, column=0, columnspan=2, sticky="w", pady=(6, 0))

    def submit():
        try:
            req = build_request(
                {name: entry.get() for name, entry in entries.items()},
                x_all=bool(x_all.get()),
                y_all=bool(y_all.get()) and not x_only,
                y_auto=bool(y_auto.get()) and not x_only,
                save=bool(save.get()),
            )
        except ValueError as exc:
            status.config(text=str(exc))
            return
        on_apply(req)
        dlg.destroy()

    def do_reset():
        on_apply(ZoomRequest(reset=True))
        dlg.destroy()

    buttons = ttk.Frame(body)
    buttons.grid(row=len(names) + 2, column=0, columnspan=2, pady=(10, 0))
    ttk.Button(buttons, text="Apply", command=submit).grid(row=0, column=0, padx=3)
    ttk.Button(buttons, text="Reset", command=do_reset).grid(row=0, column=1, padx=3)
    ttk.Button(buttons, text="Cancel", command=dlg.destroy).grid(row=0, column=2, padx=3)

    dlg.bind("<Return>", lambda _e: submit())
    dlg.bind("<Escape>", lambda _e: dlg.destroy())
    dlg.geometry(f"+{parent.winfo_rootx() + 70}+{parent.winfo_rooty() + 70}")
    next(iter(entries.values())).focus_set()
    return True


def _open_qt(fig, ax, on_apply, x_only=False):
    if not type(fig.canvas).__module__.startswith("matplotlib.backends.backend_qt"):
        return False

    from matplotlib.backends.qt_compat import QtWidgets

    dlg = QtWidgets.QDialog(fig.canvas)
    dlg.setWindowTitle(_title(ax, fig, x_only))
    form = QtWidgets.QFormLayout(dlg)

    defaults = field_defaults(ax)
    names = FIELDS[:2] if x_only else FIELDS
    edits = {}
    for name in names:
        edit = QtWidgets.QLineEdit(defaults[name])
        form.addRow(name, edit)
        edits[name] = edit

    x_all = QtWidgets.QCheckBox("apply x to every subplot")
    x_all.setChecked(True)
    form.addRow(x_all)
    y_all = QtWidgets.QCheckBox("apply y to every subplot")
    y_auto = QtWidgets.QCheckBox("y: fit to the x window")
    if not x_only:
        form.addRow(y_all)
        form.addRow(y_auto)
    save = QtWidgets.QCheckBox("also save a zoomed PNG")
    form.addRow(save)

    status = QtWidgets.QLabel("")
    status.setStyleSheet("color: #c00")
    form.addRow(status)

    # Plain buttons rather than QDialogButtonBox: the ButtonRole enum is
    # unscoped on PyQt5 and scoped on PyQt6, and `qt_compat` only shims that
    # for its own use. Three QPushButtons work identically on both bindings.
    buttons = QtWidgets.QWidget()
    row = QtWidgets.QHBoxLayout(buttons)
    apply_btn = QtWidgets.QPushButton("Apply")
    reset_btn = QtWidgets.QPushButton("Reset")
    cancel_btn = QtWidgets.QPushButton("Cancel")
    for button in (apply_btn, reset_btn, cancel_btn):
        row.addWidget(button)
    form.addRow(buttons)

    def submit():
        try:
            req = build_request(
                {name: edit.text() for name, edit in edits.items()},
                x_all=x_all.isChecked(),
                y_all=y_all.isChecked() and not x_only,
                y_auto=y_auto.isChecked() and not x_only,
                save=save.isChecked(),
            )
        except ValueError as exc:
            status.setText(str(exc))
            return
        on_apply(req)
        dlg.accept()

    def do_reset():
        on_apply(ZoomRequest(reset=True))
        dlg.accept()

    apply_btn.clicked.connect(submit)
    reset_btn.clicked.connect(do_reset)
    cancel_btn.clicked.connect(dlg.reject)
    dlg.show()
    return True


def _open_mpl(fig, ax, on_apply, x_only=False):
    """Toolkit-free fallback: a second matplotlib figure of TextBox widgets.

    Works on any interactive backend, including ones this module has no
    bespoke branch for. Closing goes through `layout.real_close()` because
    `plt.close` is a no-op by the time figures are on screen.
    """
    import matplotlib.pyplot as plt
    from matplotlib.widgets import Button, CheckButtons, TextBox

    names = FIELDS[:2] if x_only else FIELDS
    defaults = field_defaults(ax)
    flags = ["apply x to all", "apply y to all", "y: fit to x", "save PNG"]
    if x_only:
        flags = ["apply x to all", "save PNG"]

    dialog = plt.figure(figsize=(4.6, 0.5 * (len(names) + len(flags)) + 1.6))
    dialog.canvas.manager.set_window_title(_title(ax, fig, x_only))

    boxes = {}
    top = 0.94
    step = 0.5 / (len(names) + len(flags) + 2)
    for i, name in enumerate(names):
        slot = dialog.add_axes((0.42, top - (i + 1) * step * 1.6, 0.5, step * 1.1))
        boxes[name] = TextBox(slot, name + "  ", initial=defaults[name])

    check_slot = dialog.add_axes((0.06, 0.22, 0.88, top - 0.30 - len(names) * step * 1.6))
    checks = CheckButtons(check_slot, flags, [name == "apply x to all" for name in flags])

    status_slot = dialog.add_axes((0.06, 0.14, 0.88, 0.06))
    status_slot.axis("off")
    status = status_slot.text(0, 0.5, "", color="#c00", fontsize=9, va="center")

    def checked(label):
        return bool(checks.get_status()[flags.index(label)]) if label in flags else False

    def submit(_event):
        try:
            req = build_request(
                {name: box.text for name, box in boxes.items()},
                x_all=checked("apply x to all"),
                y_all=checked("apply y to all"),
                y_auto=checked("y: fit to x"),
                save=checked("save PNG"),
            )
        except ValueError as exc:
            status.set_text(str(exc))
            dialog.canvas.draw_idle()
            return
        on_apply(req)
        layout.real_close(dialog)

    def do_reset(_event):
        on_apply(ZoomRequest(reset=True))
        layout.real_close(dialog)

    apply_btn = Button(dialog.add_axes((0.10, 0.03, 0.24, 0.09)), "Apply")
    reset_btn = Button(dialog.add_axes((0.38, 0.03, 0.24, 0.09)), "Reset")
    cancel_btn = Button(dialog.add_axes((0.66, 0.03, 0.24, 0.09)), "Cancel")
    apply_btn.on_clicked(submit)
    reset_btn.on_clicked(do_reset)
    cancel_btn.on_clicked(lambda _e: layout.real_close(dialog))

    # matplotlib widgets are only kept alive by a reference; the figure is the
    # natural owner since it outlives this call.
    dialog._rtc_widgets = (boxes, checks, apply_btn, reset_btn, cancel_btn)
    # No explicit show(): pyplot already registered the figure with the active
    # GUI manager, and `Figure.show()` warns on backends without one.
    dialog.canvas.draw_idle()
    return True


# The dialog-backend interface, in fallback order.
#
# ARCH-3: the shared signature `(fig, ax, on_apply, x_only) -> bool` *is* the
# interface, and this tuple is the dispatch. A backend returns False when its
# toolkit does not own `fig.canvas`, so supporting a fourth one means writing
# a function and appending a row here — never branching inside `open_dialog`,
# which is the `#ifdef`-shaped mistake the invariant exists to prevent.
DIALOG_BACKENDS = (_open_tk, _open_qt, _open_mpl)


def open_dialog(fig, ax, on_apply, x_only=False):
    """Put a zoom dialog on screen, trying each DIALOG_BACKENDS entry in turn."""
    for impl in DIALOG_BACKENDS:
        try:
            if impl(fig, ax, on_apply, x_only=x_only):
                return True
        except Exception as exc:  # a toolkit present but unusable
            print(f"  zoom dialog: {impl.__name__} unavailable ({type(exc).__name__}: {exc})")
    print("  zoom dialog: no usable dialog backend for this matplotlib backend")
    return False


# ── attachment ────────────────────────────────────────────────────────────


def toolbar_owns_right_button(fig):
    """True when a toolbar tool (pan / zoom) has claimed the right button.

    `fig.canvas.toolbar` is None under Agg, and matplotlib reports the idle
    state as the empty-string `_Mode.NONE`, so both need handling.
    """
    toolbar = getattr(fig.canvas, "toolbar", None)
    if toolbar is None:
        return False
    return bool(str(getattr(toolbar, "mode", "") or ""))


def attach(fig, save_dir=None, opener=None):
    """Wire the right-click zoom dialog onto `fig`; returns the connection id.

    `opener(fig, ax, on_apply, x_only=...)` is the seam tests replace to
    supply values without a GUI.
    """
    opener = opener or open_dialog
    home = {axes: (tuple(axes.get_xlim()), tuple(axes.get_ylim())) for axes in fig.axes}

    def on_press(event):
        if event.button != RIGHT_BUTTON or not fig.axes:
            return
        if toolbar_owns_right_button(fig):
            return
        # Clicking the figure background instead of a panel means "the whole
        # figure": there is no meaningful per-axes y to offer, so only x.
        x_only = event.inaxes is None
        ax = event.inaxes or fig.axes[0]

        def on_apply(req):
            if x_only:
                req = replace(req, x_all=True, y_all=False, y_auto=False, ylim=None)
            apply_request(fig, ax, req, save_dir=save_dir, home=home)

        opener(fig, ax, on_apply, x_only=x_only)

    return fig.canvas.mpl_connect("button_press_event", on_press)


def attach_all(plt, save_dir=None, opener=None):
    """Attach to every figure pyplot currently holds open. Returns the count."""
    attached = 0
    for num in plt.get_fignums():
        attach(plt.figure(num), save_dir=save_dir, opener=opener)
        attached += 1
    return attached
