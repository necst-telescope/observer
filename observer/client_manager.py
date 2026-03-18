import array
import importlib
import logging
import time
import traceback
from dataclasses import dataclass, field
from functools import partial, wraps
from threading import RLock
from typing import Any, Dict, List, Optional, Set

from flask_socketio import SocketIO
from necst import qos
from necst.core.server_node import ServerNode
from rclpy.qos import QoSProfile
from rclpy.subscription import Subscription

from .stream_keys import (
    make_stream_key,
    normalize_fields,
    normalize_options,
    normalize_role_fields,
)
from .stream_profiles import transform_payload

logger = logging.getLogger(__name__)

EMIT_LIMIT_HZ = 30.0
IDLE_TTL_SEC = 20.0
STALE_AFTER_SEC = 5.0
THROTTLE_AFTER_SEC = 2.0



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
class ClientSubscription:
    stream_key: str
    topic: str
    role: str = ""
    fields: List[str] = field(default_factory=list)
    options: Dict[str, Any] = field(default_factory=dict)
    last_updated: float = 0.0


@dataclass
class Client:
    sid: str
    subscriptions: Dict[str, ClientSubscription] = field(default_factory=dict)
    connected_at: float = field(default_factory=time.time)
    last_activity_at: float = field(default_factory=time.time)


@dataclass
class TopicState:
    subscription: Subscription
    stream_keys: Set[str] = field(default_factory=set)
    client_sids: Set[str] = field(default_factory=set)
    last_rx_time: float = 0.0
    last_used_time: float = field(default_factory=time.time)
    rx_count: int = 0


@dataclass
class StreamState:
    stream_key: str
    topic: str
    role: str = ""
    fields: List[str] = field(default_factory=list)
    options: Dict[str, Any] = field(default_factory=dict)
    last_emit_time: float = 0.0
    last_rx_time: float = 0.0
    last_used_time: float = field(default_factory=time.time)
    rx_count: int = 0
    emit_count: int = 0
    throttle_count: int = 0
    last_throttle_time: float = 0.0
    client_sids: Set[str] = field(default_factory=set)


class ClientManager(ServerNode):

    _instance = None

    __clients: Dict[str, Client] = {}
    __subscribers: Dict[str, TopicState] = {}
    __streams: Dict[str, StreamState] = {}
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
            self.__cleanup_expired_subscriptions_locked(time.time())
            return list(self.__subscribers.keys())

    @return_false_on_failure
    def add_client(self, sid: str) -> bool:
        now = time.time()
        with self.__lock:
            self.__cleanup_expired_subscriptions_locked(now)
            client = self.__clients.get(sid)
            if client is None:
                self.__clients[sid] = Client(sid)
            else:
                client.last_activity_at = now
            logger.debug(self.__clients)
        return True

    @return_false_on_failure
    def remove_client(self, sid: str) -> bool:
        now = time.time()
        with self.__lock:
            client = self.__clients.get(sid)
            stream_keys = list(client.subscriptions.keys()) if client is not None else []
        for stream_key in stream_keys:
            self.remove_subscription_by_stream_key(sid, stream_key)
        with self.__lock:
            self.__clients.pop(sid, None)
            self.__cleanup_expired_subscriptions_locked(now)
        return True

    def get_client_sids(self) -> List[str]:
        with self.__lock:
            self.__cleanup_expired_subscriptions_locked(time.time())
            return sorted(self.__clients.keys())

    @return_false_on_failure
    def add_subscription(
        self,
        sid: str,
        topic: str,
        field_name: Optional[str] = None,
        role: str = "",
        options: Optional[Dict[str, Any]] = None,
    ) -> bool:
        now = time.time()
        fields = normalize_role_fields(role, [field_name] if field_name else [])
        normalized_options = normalize_options(options)
        stream_key = make_stream_key(
            topic,
            role=role,
            fields=fields,
            options=normalized_options,
        )
        with self.__lock:
            self.__cleanup_expired_subscriptions_locked(now)
            client = self.__clients.get(sid)
            if client is None:
                logger.warning(f"Subscription request from unknown client: {sid}")
                return False
            client.last_activity_at = now
            existing = client.subscriptions.get(stream_key)
            if existing is not None:
                existing.last_updated = now
                stream_state = self.__streams.get(stream_key)
                if stream_state is not None:
                    stream_state.client_sids.add(sid)
                    stream_state.last_used_time = now
                topic_state = self.__subscribers.get(topic)
                if topic_state is not None:
                    topic_state.client_sids.add(sid)
                    topic_state.last_used_time = now
                return True
            topic_state = self.__subscribers.get(topic)

        if topic_state is None:
            msgtype = get_msg_type(topic)
            if msgtype is None:
                return False
            qos_profile = get_qos_profile(topic)
            with self.__lock:
                if topic not in self.__subscribers:
                    subscription = self.create_subscription(
                        msgtype, topic, partial(self.__emit, topic), qos_profile
                    )
                    self.__subscribers[topic] = TopicState(subscription=subscription)
                topic_state = self.__subscribers[topic]
        else:
            with self.__lock:
                topic_state = self.__subscribers.get(topic)

        with self.__lock:
            client = self.__clients.get(sid)
            if client is None:
                return False
            client.subscriptions[stream_key] = ClientSubscription(
                stream_key=stream_key,
                topic=topic,
                role=role,
                fields=fields,
                options=normalized_options,
                last_updated=now,
            )
            stream_state = self.__streams.get(stream_key)
            if stream_state is None:
                stream_state = StreamState(
                    stream_key=stream_key,
                    topic=topic,
                    role=role,
                    fields=fields,
                    options=normalized_options,
                )
                self.__streams[stream_key] = stream_state
            stream_state.client_sids.add(sid)
            stream_state.last_used_time = now
            topic_state = self.__subscribers.get(topic)
            if topic_state is not None:
                topic_state.stream_keys.add(stream_key)
                topic_state.client_sids.add(sid)
                topic_state.last_used_time = now
        return True

    @return_false_on_failure
    def remove_subscription(
        self,
        sid: str,
        topic: str,
        field_name: Optional[str] = None,
        role: str = "",
        options: Optional[Dict[str, Any]] = None,
    ) -> bool:
        stream_key = make_stream_key(
            topic,
            role=role,
            fields=normalize_role_fields(role, [field_name] if field_name else []),
            options=normalize_options(options),
        )
        return self.remove_subscription_by_stream_key(sid, stream_key)

    @return_false_on_failure
    def remove_subscription_by_stream_key(self, sid: str, stream_key: str) -> bool:
        now = time.time()
        with self.__lock:
            client = self.__clients.get(sid)
            if client is None:
                return True
            client.last_activity_at = now
            subscription = client.subscriptions.pop(stream_key, None)
            if subscription is None:
                return True
            stream_state = self.__streams.get(stream_key)
            topic_state = self.__subscribers.get(subscription.topic)
            if stream_state is not None:
                stream_state.client_sids.discard(sid)
                stream_state.last_used_time = now
            if topic_state is not None:
                topic_state.client_sids.discard(sid)
                if any(sub.topic == subscription.topic for sub in client.subscriptions.values()):
                    topic_state.client_sids.add(sid)
                for candidate_sid in list(topic_state.client_sids):
                    candidate_client = self.__clients.get(candidate_sid)
                    if candidate_client is None:
                        topic_state.client_sids.discard(candidate_sid)
                        continue
                    if any(sub.topic == subscription.topic for sub in candidate_client.subscriptions.values()):
                        continue
                    topic_state.client_sids.discard(candidate_sid)
                topic_state.last_used_time = now
            self.__cleanup_expired_subscriptions_locked(now)
        return True

    def get_stream_statuses(self, stream_keys: Optional[Set[str]] = None) -> List[Dict[str, Any]]:
        now = time.time()
        with self.__lock:
            self.__cleanup_expired_subscriptions_locked(now)
            statuses: List[Dict[str, Any]] = []
            for stream_key, state in self.__streams.items():
                if stream_keys is not None and stream_key not in stream_keys:
                    continue
                last_rx_age = None if state.last_rx_time <= 0 else now - state.last_rx_time
                last_emit_age = None if state.last_emit_time <= 0 else now - state.last_emit_time
                last_throttle_age = None if state.last_throttle_time <= 0 else now - state.last_throttle_time
                status = "idle"
                if state.client_sids:
                    status = "live"
                    if last_rx_age is None or last_rx_age > STALE_AFTER_SEC:
                        status = "stale"
                    elif last_throttle_age is not None and last_throttle_age <= THROTTLE_AFTER_SEC:
                        status = "throttled"
                statuses.append(
                    {
                        "stream_key": stream_key,
                        "topic": state.topic,
                        "role": state.role,
                        "fields": list(state.fields),
                        "options": dict(state.options),
                        "status": status,
                        "client_count": len(state.client_sids),
                        "server_time": now,
                        "last_rx_time": (None if state.last_rx_time <= 0 else state.last_rx_time),
                        "last_emit_time": (None if state.last_emit_time <= 0 else state.last_emit_time),
                        "last_throttle_time": (None if state.last_throttle_time <= 0 else state.last_throttle_time),
                        "last_rx_age_sec": last_rx_age,
                        "last_emit_age_sec": last_emit_age,
                        "last_throttle_age_sec": last_throttle_age,
                        "rx_count": state.rx_count,
                        "emit_count": state.emit_count,
                        "throttle_count": state.throttle_count,
                    }
                )
            statuses.sort(key=lambda item: (item["topic"], item["role"], item["fields"]))
            return statuses

    def get_status_payload_for_sid(self, sid: str) -> Dict[str, Any]:
        with self.__lock:
            client = self.__clients.get(sid)
            stream_keys = set(client.subscriptions.keys()) if client is not None else set()
        return {
            "health": self.get_health_status(),
            "streams": self.get_stream_statuses(stream_keys=stream_keys),
        }

    def get_session_statuses(self) -> List[Dict[str, Any]]:
        now = time.time()
        with self.__lock:
            sessions: List[Dict[str, Any]] = []
            for sid, client in self.__clients.items():
                sessions.append(
                    {
                        "sid": sid,
                        "connected_age_sec": now - client.connected_at,
                        "last_activity_age_sec": now - client.last_activity_at,
                        "subscriptions": {
                            stream_key: {
                                "topic": sub.topic,
                                "role": sub.role,
                                "fields": list(sub.fields),
                                "options": dict(sub.options),
                                "last_updated_age_sec": now - sub.last_updated if sub.last_updated else None,
                            }
                            for stream_key, sub in client.subscriptions.items()
                        },
                    }
                )
            sessions.sort(key=lambda item: item["sid"])
            return sessions

    def get_health_status(self) -> Dict[str, Any]:
        now = time.time()
        with self.__lock:
            self.__cleanup_expired_subscriptions_locked(now)
            return {
                "ok": True,
                "active_clients": len(self.__clients),
                "active_topics": len(self.__subscribers),
                "active_streams": len(self.__streams),
                "idle_ttl_sec": IDLE_TTL_SEC,
                "emit_limit_hz": EMIT_LIMIT_HZ,
                "status_push_enabled": True,
                "status_push_mode": "change+heartbeat",
                "status_push_check_interval_sec": 1.0,
                "status_push_heartbeat_sec": 10.0,
            }

    def __cleanup_expired_subscriptions_locked(self, now: float) -> None:
        expired_stream_keys = [
            stream_key
            for stream_key, state in self.__streams.items()
            if (not state.client_sids) and (now - state.last_used_time >= IDLE_TTL_SEC)
        ]
        for stream_key in expired_stream_keys:
            stream_state = self.__streams.pop(stream_key, None)
            if stream_state is None:
                continue
            topic_state = self.__subscribers.get(stream_state.topic)
            if topic_state is not None:
                topic_state.stream_keys.discard(stream_key)
                topic_state.last_used_time = now

        expired_topics = [
            topic
            for topic, state in self.__subscribers.items()
            if (not state.stream_keys) and (now - state.last_used_time >= IDLE_TTL_SEC)
        ]
        subscriptions_to_destroy = [
            (topic, self.__subscribers[topic].subscription) for topic in expired_topics
        ]
        for topic in expired_topics:
            self.__subscribers.pop(topic, None)
        for topic, subscription in subscriptions_to_destroy:
            logger.info(f"Destroying idle subscription to {topic}")
            self.destroy_subscription(subscription)

    def __emit(self, topic: str, msg: Any) -> None:
        now = time.time()
        with self.__lock:
            topic_state = self.__subscribers.get(topic)
            if topic_state is None:
                return
            topic_state.last_rx_time = now
            topic_state.last_used_time = now
            topic_state.rx_count += 1
            stream_snapshots = []
            for stream_key in list(topic_state.stream_keys):
                stream_state = self.__streams.get(stream_key)
                if stream_state is None:
                    topic_state.stream_keys.discard(stream_key)
                    continue
                stream_state.last_rx_time = now
                stream_state.last_used_time = now
                stream_state.rx_count += 1
                if not stream_state.client_sids:
                    continue
                if stream_state.last_emit_time > now - 1 / EMIT_LIMIT_HZ:
                    stream_state.throttle_count += 1
                    stream_state.last_throttle_time = now
                    continue
                stream_snapshots.append(
                    {
                        "stream_key": stream_key,
                        "role": stream_state.role,
                        "fields": list(stream_state.fields),
                        "options": dict(stream_state.options),
                        "sids": list(stream_state.client_sids),
                    }
                )

        if not stream_snapshots:
            return

        try:
            raw_data = serialize(msg)
        except Exception:
            logger.error(traceback.format_exc())
            return

        socket = self.__socket
        if socket is None:
            logger.error(f"Socket not attached, cannot emit incoming message: {msg}")
            return

        emitted_stream_keys: Set[str] = set()
        for snapshot in stream_snapshots:
            try:
                payload = transform_payload(
                    raw_data,
                    role=snapshot["role"],
                    fields=snapshot["fields"],
                    options=snapshot["options"],
                    fallback_time=now,
                )
            except Exception:
                logger.error(traceback.format_exc())
                continue
            if not payload:
                continue
            for sid in snapshot["sids"]:
                socket.emit(
                    "ros2-message",
                    {
                        "stream_key": snapshot["stream_key"],
                        "topic_name": topic,
                        "data": payload,
                        "role": snapshot["role"],
                    },
                    to=sid,
                    namespace="/qlook",
                )
            emitted_stream_keys.add(snapshot["stream_key"])

        if not emitted_stream_keys:
            return
        with self.__lock:
            for stream_key in emitted_stream_keys:
                stream_state = self.__streams.get(stream_key)
                if stream_state is not None:
                    stream_state.last_emit_time = now
                    stream_state.emit_count += 1
