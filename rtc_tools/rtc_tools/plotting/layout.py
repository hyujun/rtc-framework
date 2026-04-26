"""Figure layout helpers — subplot grid sizing + backend setup + save/show.

Backend rule:
  - If save_dir is set and --show is OFF, force the Agg backend (headless,
    fast PNG only) BEFORE matplotlib.pyplot is imported.
  - If --show is ON, plot functions' `plt.close()` is monkey-patched to a no-op
    so all figures survive until the final `plt.show()` in main().
"""

import math
from pathlib import Path

# Late-bound; configure_backend() must run before any plot uses this.
plt = None


def configure_backend(save_only):
    """Pick matplotlib backend based on whether we are saving headlessly.

    save_only=True  → use 'Agg' (no GUI dependency).
    save_only=False → keep default interactive backend so plt.show() works.

    Returns the imported pyplot module so callers can use it directly.
    """
    if save_only:
        import matplotlib

        matplotlib.use("Agg")
    import matplotlib.pyplot as _plt

    global plt
    plt = _plt
    return _plt


def disable_close():
    """Make plt.close() a no-op so figures survive until main()'s plt.show()."""
    if plt is not None:
        plt.close = lambda *a, **kw: None


def auto_subplot_grid(n, max_cols=None):
    """Calculate optimal (nrows, ncols) for n subplots.

    Picks the layout closest to square with ncols >= nrows.
    Allows a small number of empty cells to avoid tall 1-column layouts.

    Examples:
      1 → (1,1)   2 → (1,2)   3 → (1,3)   4 → (2,2)
      5 → (2,3)   6 → (2,3)   7 → (3,3)   8 → (2,4)
      9 → (3,3)  10 → (2,5)  12 → (3,4)  16 → (4,4)
    """
    if n <= 0:
        return (1, 1)
    if n <= 3:
        return (1, n)

    if max_cols is not None:
        ncols = min(n, max_cols)
        nrows = math.ceil(n / ncols)
        return (nrows, ncols)

    ncols = math.ceil(math.sqrt(n))
    nrows = math.ceil(n / ncols)
    if nrows > ncols:
        nrows, ncols = ncols, nrows
        nrows = math.ceil(n / ncols)
    return (nrows, ncols)


def save_or_show(fig, save_dir, filename):
    """Save fig to save_dir/filename or call plt.show(); always closes fig.

    `plt.close()` is a no-op when --show is active (see disable_close()).
    """
    plt.tight_layout()
    if save_dir:
        path = Path(save_dir) / filename
        plt.savefig(path, dpi=300, bbox_inches="tight")
        print(f"Saved: {path}")
    else:
        plt.show()
    plt.close()
