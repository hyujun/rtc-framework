"""pytest configuration for the rtc_tools tests.

``colcon.pkg`` hands pytest ``-n 4`` (pytest-xdist: four worker processes).
xdist is an accelerator, not a requirement: a host without it must still run
these tests, one at a time, and reach the same verdict. pytest rejects an
option nobody registered, so when the xdist plugin is not loaded this file
registers ``-n`` itself, ignores its value, and says so in the report header.

PANDAS COPY-ON-WRITE. The tools run under the workspace venv (pandas 3, where
copy-on-write is the only mode: ``Series.to_numpy()`` hands back a READ-ONLY
view, and writing into it raises). ``colcon test`` runs them under the system
python (pandas 2.1, copy-on-write off: the same call hands back a writeable
array). A write into such a view therefore passed every colcon run and broke
``plot_rtc_log`` on every catching_diag.csv (2026-10-08, #744 — found by
running the suite under the venv by hand). Under a pandas that still has the
switch, this file turns copy-on-write ON, so that the colcon lane fails where
the runtime does.
"""

import pandas as pd

_XDIST_MISSING = "rtc_tools_xdist_missing"
_COW_FORCED = False


def pytest_configure(config):
    global _COW_FORCED
    # pandas 3 has no switch (copy-on-write is the only mode, and the option
    # is gone or a no-op); pandas 2.x has one, off by default.
    if int(pd.__version__.split(".")[0]) < 3:
        pd.options.mode.copy_on_write = True
        _COW_FORCED = True


def pytest_addoption(parser, pluginmanager):
    # hasplugin, not `import xdist`: an installed xdist that was switched off
    # (`-p no:xdist`) has registered no `-n` either.
    if pluginmanager.hasplugin("xdist"):
        return
    # _addoption, as xdist itself calls it: the public addoption() refuses a
    # lowercase short option ("lowercase shortoptions reserved").
    parser.getgroup("rtc_tools")._addoption(
        "-n",
        "--numprocesses",
        action="store",
        default=None,
        dest=_XDIST_MISSING,
        help="ignored: pytest-xdist is not loaded, the tests run one at a time",
    )


def pytest_report_header(config):
    lines = []
    if config.getoption(_XDIST_MISSING, None) is not None:
        lines.append(
            "rtc_tools: pytest-xdist is not loaded -- '-n' is ignored and the tests run "
            "one at a time (sudo apt install python3-pytest-xdist)"
        )
    if _COW_FORCED:
        lines.append(
            f"rtc_tools: pandas {pd.__version__} -- copy-on-write turned ON for the tests, "
            "as the venv's pandas 3 behaves"
        )
    return lines or None
