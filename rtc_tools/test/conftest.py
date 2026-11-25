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

ONE SKIP, ONE MODULE. Several modules end their imports with a module-level
``pytest.importorskip("mujoco")``: the system python of ``colcon test`` has no
mujoco, so those modules skip there. ROS Jazzy's ``launch_testing`` plugin
(3.4.13) imports every ``test_*.py`` from its ``pytest_pycollect_makemodule``
hook to look for a launch test, and guards the import with ``except
Exception`` only. ``Skipped`` is a ``BaseException``, so it escapes the hook
and pytest skips the DIRECTORY being collected: ``colcon test`` reported
"0 items / 1 skipped", pytest exited 5 and colcon counted that as a pass
(2026-11-25, every rtc_tools test unrun). So this file builds the plain
``Module`` first for every test file that is not a launch test, and the
plugin's hook is never asked for it; a file that defines
``generate_test_description`` is still left to the plugin, which is what
collects its tests. And a run over the whole suite that collects nothing exits as a
failure, not as 5 -- "did not run" is not "passed".
"""

from pathlib import Path

import pandas as pd
import pytest

_TEST_DIR = Path(__file__).resolve().parent

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


@pytest.hookimpl(tryfirst=True)
def pytest_pycollect_makemodule(module_path, parent):
    # firstresult hook: a Module returned here ends the call before the
    # launch_testing plugin imports the file outside a collector of its own.
    if module_path.suffix != ".py" or not module_path.name.startswith("test_"):
        return None
    # A launch test is the plugin's to collect: as a plain Module it would
    # yield no test and pass unrun. Read the text, do not import it.
    if "generate_test_description" in module_path.read_text(encoding="utf-8"):
        return None
    return pytest.Module.from_parent(parent, path=module_path)


def _covers_the_whole_suite(config) -> bool:
    if config.getoption("keyword") or config.getoption("markexpr"):
        return False
    roots = {Path(str(config.rootpath)).resolve(), _TEST_DIR}
    return all(Path(arg.split("::")[0]).resolve() in roots for arg in config.args)


def pytest_sessionfinish(session, exitstatus):
    if hasattr(session.config, "workerinput"):  # xdist worker: the controller decides
        return
    if exitstatus == pytest.ExitCode.NO_TESTS_COLLECTED and _covers_the_whole_suite(
        session.config
    ):
        reporter = session.config.pluginmanager.get_plugin("terminalreporter")
        if reporter is not None:
            reporter.write_line(
                "rtc_tools: the whole suite collected no test -- failing the run "
                "(see test/conftest.py, ONE SKIP, ONE MODULE)",
                red=True,
            )
        session.exitstatus = pytest.ExitCode.TESTS_FAILED


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


def robots_dir() -> Path | None:
    """``robot_descriptions/robots``: ament share first, then the in-source tree.

    Shared by the gates that fire on the shipped robot assets
    (``test_real_model_pairs.py``, ``test_vendored_manifests.py``), so the two
    cannot drift onto different copies of the same directory.
    """
    try:
        from ament_index_python.packages import get_package_share_directory

        share = Path(get_package_share_directory("robot_descriptions")) / "robots"
        if (share / "model_pairs.yaml").is_file():
            return share
    except Exception:
        pass
    in_source = Path(__file__).resolve().parents[2] / "robot_descriptions" / "robots"
    if (in_source / "model_pairs.yaml").is_file():
        return in_source
    return None
