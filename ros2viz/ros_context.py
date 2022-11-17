# from contextlib import contextmanager
from typing import Generator

import rclpy
from rclpy.node import Node


class ros2node:
    def __enter__(self) -> Node:
        rclpy.init()
        self.node = rclpy.create_node("ros2viz_node")
        return self.node

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.node.destroy_node()
        rclpy.shutdown()
        if exc_type is not None:
            raise
