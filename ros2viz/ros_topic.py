import importlib
import logging
from typing import Any, Optional

from flask_socketio import SocketIO
from rclpy.duration import Duration
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    LivelinessPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


from .ros_context import ros2node

logger = logging.getLogger(__name__)


class SubscribeTo:
    def __init__(self, topic_name: str, sock: SocketIO) -> None:
        self.topic_name = topic_name
        self.node = ros2node()
        self.subscription = None
        self.sock = sock

    def start(self) -> None:
        msg_type = get_msg_type(self.topic_name)
        qos = get_qos_profile(self.topic_name)
        if msg_type is None:
            return  # emit error
        self.subscription = self.node.create_subscription(
            msg_type, self.topic_name, self.callback, qos
        )

    def callback(self, msg: Any) -> None:
        self.sock.emit(
            "ros-message", {"topic": self.topic_name, "msg": msg_to_dict(msg)}
        )

    def stop(self) -> None:
        if self.subscription is not None:
            self.node.destroy_subscription(self.subscription)


def msg_to_dict(msg: Any) -> str:
    return {k: getattr(msg, k) for k in msg.get_fields_and_field_types().keys()}


def get_qos_profile(topic_name: str) -> QoSProfile:
    node = ros2node()
    topic_info = node.get_publishers_info_by_topic(topic_name)
    if not topic_info:
        topic_info = node.get_subscriptions_info_by_topic(topic_name)
    if not topic_info:
        logger.warning(f"QoS for requested topic {topic_name!r} not found")
        return QoSProfile(
            depth=10,
            deadline=Duration(),
            history=HistoryPolicy.KEEP_LAST,
            lifespan=Duration(),
            liveliness=LivelinessPolicy.AUTOMATIC,
            liveliness_lease_duration=Duration(),
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )

    qos_info = map(lambda t: t.qos_profile, topic_info)
    return next(qos_info)  # No ``all`` check is performed.


def get_msg_type(topic_name: str) -> Optional[Any]:
    node = ros2node()
    topics = dict(node.get_topic_names_and_types())

    info = topics.get(topic_name, None)
    if info is None:
        logger.warning(f"Message type for requested topic {topic_name!r} not found")
        return
    msg_type_path, *_ = info

    module_name, msg_name = msg_type_path.replace("/", ".").rsplit(".", 1)
    module = importlib.import_module(module_name)
    msg_type = getattr(module, msg_name)
    return msg_type
