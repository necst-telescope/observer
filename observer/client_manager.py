import array
import importlib
import logging
import time
import traceback
from dataclasses import dataclass, field
from functools import partial, wraps
from typing import Any, Dict, List, Optional, Tuple

from flask_socketio import SocketIO
from necst import qos
from necst.core.server_node import ServerNode
from rclpy.qos import QoSProfile
from rclpy.subscription import Subscription

logger = logging.getLogger(__name__)


def get_msg_type(topic_name: str) -> Optional[Any]:
    node = ClientManager()
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


def get_qos_profile(topic_name: str) -> QoSProfile:
    node = ClientManager()
    return qos.adaptive(topic_name, node)


def serialize(msg: Any) -> str:
    def get(obj: Any, key: str) -> Any:
        attr = getattr(obj, key)
        return list(attr) if isinstance(attr, (array.array, list)) else attr

    return {k: get(msg, k) for k in msg.get_fields_and_field_types().keys()}


def return_false_on_failure(func):
    @wraps(func)
    def new_func(*args, **kwargs):
        try:
            return func(*args, **kwargs)
        except Exception:
            logger.error(traceback.format_exc())
            return False

    return new_func


@dataclass
class Client:
    sid: str
    subscriptions: List[str] = field(default_factory=list)

    def subscribes_to(self, topic: str) -> bool:
        return topic in self.subscriptions


class ClientManager(ServerNode):

    _instance = None

    __clients: Dict[str, Client] = {}
    __subscribers: Dict[str, Tuple[Subscription, float]] = {}
    __socket: Optional[SocketIO] = None

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, socket: SocketIO = None) -> None:
        if getattr(self, "executor") is None:
            super().__init__("ros2viz")
            self.start_server()
        self.__socket = socket or self.__socket

    @property
    def current_subscriptions(self) -> List[str]:
        return list(self.__subscribers.keys())

    @property
    def client_count(self) -> int:
        return len(self.__clients)

    @return_false_on_failure
    def add_client(self, sid: str) -> bool:
        self.__clients[sid] = Client(sid)
        logger.debug(self.__clients)
        return True

    @return_false_on_failure
    def remove_client(self, sid: str) -> bool:
        for topic in self.__clients[sid].subscriptions:
            self.remove_subscription(sid, topic)
        del self.__clients[sid]
        return True

    @return_false_on_failure
    def add_subscription(self, sid: str, topic: str) -> bool:
        if topic not in self.current_subscriptions:
            msgtype = get_msg_type(topic)
            qos = get_qos_profile(topic)
            self.__subscribers[topic] = (
                self.create_subscription(
                    msgtype, topic, partial(self.__emit, topic), qos
                ),
                time.time(),
            )
        self.__clients[sid].subscriptions.append(topic)
        return True

    @return_false_on_failure
    def remove_subscription(self, sid: str, topic: str) -> bool:
        self.__clients[sid].subscriptions.remove(topic)
        for client in self.__clients.values():
            if topic in client.subscriptions:
                return
        logger.info(f"Destroying subscription to {topic}")
        self.destroy_subscription(self.__subscribers[topic][0])
        del self.__subscribers[topic]
        return True

    def __emit(self, topic: str, msg: Any) -> None:
        if self.__subscribers[topic][1] > time.time() - 1 / 30:
            return

        data = serialize(msg)
        if self.__socket is None:
            logger.error(f"Socket not attached, cannot emit incoming message: {msg}")
            return
        self.__socket.emit(
            "ros2-message",
            {"topic_name": topic, "data": data},
            to=topic,
            namespace="/qlook",
        )
