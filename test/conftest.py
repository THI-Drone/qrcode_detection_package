import pytest
import rclpy


def pytest_sessionstart(session):
    rclpy.init(args=[])


def pytest_sessionfinish(session, exitstatus):
    rclpy.shutdown()
