"""demo_controller_gui modules.

This package was split out of scripts/demo_controller_gui.py (Phase 0a) so
the GUI source can grow into smaller, single-responsibility modules without
turning a single 2.7 kLOC file into a maintenance hazard. The current split
is by file only — module-level state still lives on DemoControllerGUI.

``app`` (Tk and rclpy) is imported on first use of ``main`` or
``DemoControllerGUI``, not with the package: the headless sequence CLI
(``integrated_bringup.demo_sequence``) imports the Tk-free modules here
(``sequence``, ``preset_speed``, ``discovery``) on a machine with no display.
"""

__all__ = ["DemoControllerGUI", "main"]


def main(*args, **kwargs):
    from .app import main as app_main

    return app_main(*args, **kwargs)


def __getattr__(name: str):
    if name == "DemoControllerGUI":
        from .app import DemoControllerGUI

        return DemoControllerGUI
    raise AttributeError(f"module {__name__!r} has no attribute {name!r}")
