"""Shared fixtures: one rclpy context for the tests that talk ROS.

The tests with wall-clock gates poll until a deadline instead of reading one
sample, and every ROS test uses its own topic names (``uid``) so that a
neighbour's leftover publisher cannot feed it.
"""

from __future__ import annotations

import itertools
import os

import pytest
import rclpy

_ids = itertools.count()


@pytest.fixture(scope="session")
def ros():
    rclpy.init()
    yield
    rclpy.try_shutdown()


@pytest.fixture
def uid():
    return f"t{os.getpid()}_{next(_ids)}"
