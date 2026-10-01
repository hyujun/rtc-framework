"""pytest configuration for the rtc_tools tests.

``colcon.pkg`` hands pytest ``-n 4`` (pytest-xdist: four worker processes).
xdist is an accelerator, not a requirement: a host without it must still run
these tests, one at a time, and reach the same verdict. pytest rejects an
option nobody registered, so when the xdist plugin is not loaded this file
registers ``-n`` itself, ignores its value, and says so in the report header.
"""

_XDIST_MISSING = "rtc_tools_xdist_missing"


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
    if config.getoption(_XDIST_MISSING, None) is not None:
        return (
            "rtc_tools: pytest-xdist is not loaded -- '-n' is ignored and the tests run "
            "one at a time (sudo apt install python3-pytest-xdist)"
        )
    return None
