from threading import Event, Thread

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node


class ros2node(Node):
    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self) -> None:
        super().__init__("ros2viz")


class ros2env:
    def __enter__(self) -> None:
        rclpy.init()
        self.thread = Thread(target=self.run, daemon=True)
        self.event = Event()
        self.node = ros2node()
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self.thread.start()

    def run(self) -> None:
        while not self.event.is_set():
            self.executor.spin_once(0.05)

    def __exit__(self, exc_type, exc_value, traceback) -> None:
        self.event.set()
        self.thread.join()
        [node.destroy_node() for node in self.executor.get_nodes()]
        [self.executor.remove_node(node) for node in self.executor.get_nodes()]
        self.executor.shutdown()
        rclpy.shutdown()
        if exc_type is not None:
            raise
