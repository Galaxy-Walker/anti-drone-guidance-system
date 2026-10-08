"""ROS 节点测试共用的上下文初始化，保持每个测试模块独立。"""

from __future__ import annotations

import pytest
import rclpy


@pytest.fixture(scope="module", autouse=True)
def ros_context():
    rclpy.init()
    yield
    rclpy.shutdown()
