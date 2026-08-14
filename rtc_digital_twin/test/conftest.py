# Test DDS domain isolation for this package (issue #401).
#
# `colcon test` runs PACKAGES in parallel, so any test that opens a DDS
# participant on the shared default domain 0 sits on the same discovery bus
# and the same Fast DDS shared-memory port objects as every other unclaimed
# package running beside it. When one of those is killed abnormally -- which
# ctest does at its 60 s timeout -- the survivors can block inside rmw
# endpoint create/destroy until they are killed too, leaving no result file.
#
# This package is `ament_python`: it has no CMakeLists, so the
# `ament_add_test(ENV ROS_DOMAIN_ID=...)` idiom the C++ packages use is not
# available here. pytest imports conftest.py before it imports any test
# module, which makes this the one place the id can be set early enough for
# rclpy.init() to read it.
#
# The value must stay a literal: repo_scripts/scripts/validate_test_domains.py
# reads this file as text to keep the allocation collision-free across
# packages, and cannot evaluate anything else. Run that script with --list to
# see the current allocation; it is derived from the sources, never restated.
import os

os.environ["ROS_DOMAIN_ID"] = "58"
