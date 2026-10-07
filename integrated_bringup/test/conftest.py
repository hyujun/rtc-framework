"""pytest configuration for the integrated_bringup python tests.

PANDAS COPY-ON-WRITE. The eval tools under tools/ (summarize, tc_vector,
unit_report) run under the workspace venv, where pandas 3 makes copy-on-write
the only mode: ``Series.to_numpy()`` hands back a READ-ONLY view and a write
into it raises. ``colcon test`` runs these tests under the system python, whose
pandas 2.1 hands back a writeable array for the same call. Such a write therefore
passes every colcon run and fails at the first real use (rtc_tools, 2026-10-08,
#744). Under a pandas that still has the switch, it is turned ON here, so that
the colcon lane fails where the runtime does. The same guard is in
rtc_tools/test/conftest.py.
"""

import pandas as pd

_COW_FORCED = False


def pytest_configure(config):
    global _COW_FORCED
    # pandas 3 has no switch (copy-on-write is the only mode); pandas 2.x has
    # one, off by default.
    if int(pd.__version__.split(".")[0]) < 3:
        pd.options.mode.copy_on_write = True
        _COW_FORCED = True


def pytest_report_header(config):
    if _COW_FORCED:
        return (
            f"integrated_bringup: pandas {pd.__version__} -- copy-on-write turned ON for the "
            "tests, as the venv's pandas 3 behaves"
        )
    return None
