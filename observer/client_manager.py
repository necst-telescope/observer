import array
import importlib
import logging
import time
import traceback
from dataclasses import dataclass, field
from functools import partial, wraps
from threading import RLock
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
        return None
    msg_type_path, *_ = info

    module_name, msg_name = msg_type_path.replace("/", ".").rsplit(".", 1)
    module = importlib.import_module(module_name)
    msg_type = getattr(module, msg_name)
    return msg_type


def get_qos_profile(topic_name: str) -> QoSProfile:
    node = ClientManager()
    return qos.adaptive(topic_name, node)


def serialize(msg: Any) -> Dict[str, Any]:
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
    __lock = RLock()

    def __new__(cls, *args, **kwargs):
        if cls._instance is None:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, socket: SocketIO = None) -> None:
        if getattr(self, "executor", None) is None:
            super().__init__("ros2viz")
            self.start_server()
        self.__socket = socket or self.__socket

    @property
    def current_subscriptions(self) -> List[str]:
        with self.__lock:
            return list(self.__subscribers.keys())

    @return_false_on_failure
    def add_client(self, sid: str) -> bool:
        with self.__lock:
            self.__clients[sid] = Client(sid)
            logger.debug(self.__clients)
        return True

    @return_false_on_failure
    def remove_client(self, sid: str) -> bool:
        with self.__lock:
            client = self.__clients.get(sid)
            topics = list(client.subscriptions) if client is not None else []
        for topic in topics:
            self.remove_subscription(sid, topic)
        with self.__lock:
            self.__clients.pop(sid, None)
        return True

    @return_false_on_failure
    def add_subscription(self, sid: str, topic: str) -> bool:
        with self.__lock:
            client = self.__clients.get(sid)
            if client is None:
                logger.warning(f"Subscription request from unknown client: {sid}")
                return False
            if topic in client.subscriptions:
                return True
            if topic in self.__subscribers:
                client.subscriptions.append(topic)
                return True

        msgtype = get_msg_type(topic)
        if msgtype is None:
            return False
        qos_profile = get_qos_profile(topic)

        with self.__lock:
            client = self.__clients.get(sid)
            if client is None:
                return False
            if topic in client.subscriptions:
                return True
            if topic not in self.__subscribers:
                subscription = self.create_subscription(
                    msgtype, topic, partial(self.__emit, topic), qos_profile
                )
                self.__subscribers[topic] = (subscription, 0.0)
            client.subscriptions.append(topic)
        return True

    @return_false_on_failure
    def remove_subscription(self, sid: str, topic: str) -> bool:
        with self.__lock:
            client = self.__clients.get(sid)
            if client is None:
                return True
            while topic in client.subscriptions:
                client.subscriptions.remove(topic)
            topic_still_used = any(
                topic in other.subscriptions
                for other_sid, other in self.__clients.items()
                if other_sid != sid
            )
            if topic_still_used:
                return True
            subscription_info = self.__subscribers.get(topic)

        if subscription_info is not None:
            logger.info(f"Destroying subscription to {topic}")
            self.destroy_subscription(subscription_info[0])
            with self.__lock:
                self.__subscribers.pop(topic, None)
        return True

    def __emit(self, topic: str, msg: Any) -> None:
        now = time.time()
        with self.__lock:
            subscription_info = self.__subscribers.get(topic)
            if subscription_info is None:
                return
            _, last_emit = subscription_info
            if last_emit > now - 1 / 30:
                return
            socket = self.__socket
            self.__subscribers[topic] = (subscription_info[0], now)

        try:
            data = serialize(msg)
        except Exception:
            logger.error(traceback.format_exc())
            return

        if socket is None:
            logger.error(f"Socket not attached, cannot emit incoming message: {msg}")
            return
        socket.emit(
            "ros2-message",
            {"topic_name": topic, "data": data},
            to=topic,
            namespace="/qlook",
        )
