"""demo_controller_gui modules.

This package was split out of scripts/demo_controller_gui.py (Phase 0a) so
the GUI source can grow into smaller, single-responsibility modules without
turning a single 2.7 kLOC file into a maintenance hazard. The current split
is by file only — module-level state still lives on DemoControllerGUI.
"""

from .app import DemoControllerGUI, main

__all__ = ["DemoControllerGUI", "main"]
