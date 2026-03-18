import logging
import os
import re
import time
import traceback
from pathlib import Path
from threading import Lock
from typing import Any, Dict


import neclib
import rclpy
import tomlkit
from flask import (
    Flask,
    Response,
    escape,
    jsonify,
    redirect,
    render_template,
    request,
    url_for,
)
from flask_socketio import SocketIO
from neclib.core import environ

from .address import get_ip_address
from .client_manager import ClientManager, get_msg_type

logger = logging.getLogger(__name__)
ch = logging.StreamHandler()
_fmt = "%(asctime)-s: [%(levelname)-s: %(filename)s#L%(lineno)s] %(message)s"
fmt = logging.Formatter(_fmt)
ch.setFormatter(fmt)
rootLogger = logging.getLogger()
rootLogger.setLevel(logging.INFO)
chs = filter(lambda x: isinstance(x, logging.StreamHandler), rootLogger.handlers)
[rootLogger.handlers.remove(_ch) for _ch in chs]
rootLogger.addHandler(ch)

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")

app.url_map.strict_slashes = False
_status_pusher_started = False
_status_pusher_lock = Lock()
_status_cache_lock = Lock()
_status_signature_by_sid: Dict[str, Any] = {}
_status_last_sent_at_by_sid: Dict[str, float] = {}
STATUS_PUSH_CHECK_INTERVAL_SEC = 1.0
STATUS_PUSH_HEARTBEAT_SEC = 10.0


def _stream_role_topics(role: str, topics):
    if role in {"total_power", "spectrum_decimated"}:
        topics = [topic for topic in topics if "quick_spectra" in topic[0]]
        if not topics:
            logger.info("There is no spectra data in ROS topics.")
    elif role == "2d-plot":
        topics = [topic for topic in topics if re.search("/encoder", topic[0])]
        if not topics:
            logger.info("There is no data that can be plotted in 2-D in ROS topics.")
    elif role == "sis_iv":
        topics = [topic for topic in topics if re.search("/sis_bias", topic[0])]
        if not topics:
            logger.info("There is no data that can be plotted in 2-D in ROS topics.")
    return topics



def build_status_signature(payload: Dict[str, Any]) -> Any:
    health = payload.get("health") or {}
    streams = payload.get("streams") or []
    stream_signature = []
    for stream in streams:
        stream_signature.append(
            (
                stream.get("stream_key"),
                stream.get("status"),
                stream.get("client_count"),
                tuple(stream.get("fields") or []),
                tuple(sorted((stream.get("options") or {}).items())),
                stream.get("role"),
                stream.get("topic"),
            )
        )
    stream_signature.sort()
    return {
        "health": (
            health.get("active_clients"),
            health.get("active_topics"),
            health.get("active_streams"),
            health.get("status_push_mode"),
            health.get("status_push_heartbeat_sec"),
        ),
        "streams": tuple(stream_signature),
    }



def emit_status_to_sid(sid: str, *, force: bool = False) -> bool:
    manager = ClientManager(socketio)
    payload = manager.get_status_payload_for_sid(sid)
    signature = build_status_signature(payload)
    now = time.time()
    with _status_cache_lock:
        last_signature = _status_signature_by_sid.get(sid)
        last_sent_at = _status_last_sent_at_by_sid.get(sid, 0.0)
        should_emit = force or (signature != last_signature) or ((now - last_sent_at) >= STATUS_PUSH_HEARTBEAT_SEC)
        if not should_emit:
            return False
        _status_signature_by_sid[sid] = signature
        _status_last_sent_at_by_sid[sid] = now
    socketio.emit(
        "ros2-status",
        payload,
        to=sid,
        namespace="/qlook",
    )
    return True



def forget_status_sid(sid: str) -> None:
    with _status_cache_lock:
        _status_signature_by_sid.pop(sid, None)
        _status_last_sent_at_by_sid.pop(sid, None)



def status_push_loop() -> None:
    while True:
        socketio.sleep(STATUS_PUSH_CHECK_INTERVAL_SEC)
        manager = ClientManager(socketio)
        active_sids = set(manager.get_client_sids())
        with _status_cache_lock:
            known_sids = set(_status_signature_by_sid.keys()) | set(_status_last_sent_at_by_sid.keys())
        for sid in sorted(known_sids - active_sids):
            forget_status_sid(sid)
        for sid in sorted(active_sids):
            emit_status_to_sid(sid)



def ensure_status_pusher_started() -> None:
    global _status_pusher_started
    with _status_pusher_lock:
        if _status_pusher_started:
            return
        socketio.start_background_task(status_push_loop)
        _status_pusher_started = True


@app.route("/")
def index() -> Response:
    return render_template("index.html")


@app.route("/qlook")
def qlook() -> str:
    return render_template("qlook/index.html")


@app.route("/healthz")
def healthz() -> Response:
    return jsonify(ClientManager(socketio).get_health_status())


@app.route("/debug/streams")
def debug_streams() -> Response:
    manager = ClientManager(socketio)
    payload = manager.get_health_status()
    payload["streams"] = manager.get_stream_statuses()
    return jsonify(payload)


@app.route("/debug/sessions")
def debug_sessions() -> Response:
    manager = ClientManager(socketio)
    payload = manager.get_health_status()
    payload["sessions"] = manager.get_session_statuses()
    return jsonify(payload)


@app.route("/config/<filename>")
def config_file(filename: str) -> str:
    path = Path.home() / ".necst" / filename
    if not path.exists():
        os.environ.pop(environ.necst_root.name, None)
        neclib.configure()

    try:
        return path.read_text()
    except Exception:
        return ""


@app.route("/config")
def config() -> str:
    path = Path.home() / ".necst"
    if not path.exists():
        os.environ.pop(environ.necst_root.name, None)
        neclib.configure()

    files = filter(lambda x: x.is_file(), path.glob("**/*"))
    file_href = [
        {"name": f.name, "path": url_for("config_edit", filename=f.name)} for f in files
    ]
    return render_template("config/index.html", files=file_href)


@app.route("/config/edit/<path:filename>", methods=["GET", "POST"])
def config_edit(filename: str) -> str:
    path = Path.home() / ".necst" / filename

    if request.method == "GET":
        if not path.exists():
            os.environ.pop(environ.necst_root.name, None)
            neclib.configure()

        try:
            content = path.read_text()
        except Exception:
            content = ""
        return render_template("config/edit.html", content=escape(content))

    elif request.method == "POST":
        # TODO: Restrict who can edit the file, like `PrivilegedNode`.
        content = request.form["content"]
        try:
            tomlkit.parse(content)  # Validate file format.
            path.write_text(content)
            return redirect(url_for("config"), code=302)
        except Exception:
            logger.warning(traceback.format_exc())
            return traceback.format_exc()


@socketio.on("connect", namespace="/qlook")
def connect(auth=None) -> bool:
    logger.info(f"New Connection: {request.sid}")
    ensure_status_pusher_started()
    success = ClientManager(socketio).add_client(request.sid)
    if success:
        emit_status_to_sid(request.sid, force=True)
    return success


@socketio.on("disconnect", namespace="/qlook")
def disconnect() -> bool:
    logger.info(f"Disconnected: {request.sid}")
    for _ in range(10):
        if ClientManager(socketio).remove_client(request.sid):
            break
        time.sleep(0.1)
    forget_status_sid(request.sid)
    return True


@socketio.on("ros2-topic-list-request", namespace="/qlook")
def ros2_topic_list_request(json: Dict[str, str]) -> None:
    logger.info(f"Got 'ros2-topic-list-request' from {request.sid}")
    topics = ClientManager(socketio).get_topic_names_and_types()
    role = (json or {}).get("role", "")
    topics = _stream_role_topics(role, topics)

    topic_split = {}
    for topic_name, *_ in topics:
        sp = re.split(r"(?=/)", topic_name, 3)
        if len(sp) != 4:
            topic_split[topic_name] = {
                "system": "",
                "observatory": "",
                "topic": topic_name,
                "display_topic": topic_name,
            }
        else:
            topic_split[topic_name] = {
                "system": sp[1],
                "observatory": sp[2],
                "topic": sp[3],
                "display_topic": sp[3],
            }

    socketio.emit(
        "ros2-topic-list",
        {"topic_split": topic_split},
        to=request.sid,
        namespace="/qlook",
    )


@socketio.on("ros2-topic-field-request", namespace="/qlook")
def ros2_topic_field_request(json: Dict[str, str]) -> None:
    logger.info(f"Got 'ros2-topic-field-request' from {request.sid}")
    topic_info = (json or {}).get("topic_name", [])
    topic_name = "".join(topic_info)
    msg_type = get_msg_type(topic_name)
    if msg_type is None:
        socketio.emit(
            "ros2-topic-field",
            {"error": f"Cannot find message type for {topic_name!r}"},
            to=request.sid,
            namespace="/qlook",
        )
        return
    socketio.emit(
        "ros2-topic-field",
        {"topic_name": topic_name, "fields": msg_type.get_fields_and_field_types()},
        to=request.sid,
        namespace="/qlook",
    )


@socketio.on("ros2-subscribe-request", namespace="/qlook")
def ros2_subscribe_request(json: Dict[str, Any]) -> None:
    logger.info(f"Got 'ros2-subscribe-request' from {request.sid}")
    topic_name = (json or {}).get("topic_name")
    field_name = (json or {}).get("field_name")
    role = (json or {}).get("role", "")
    options = (json or {}).get("options") or {}
    success = False
    if topic_name is not None:
        success = ClientManager(socketio).add_subscription(
            request.sid,
            topic_name,
            field_name=field_name,
            role=role,
            options=options,
        )
    socketio.emit(
        "ros2-subscribe",
        {"success": success},
        to=request.sid,
        namespace="/qlook",
    )
    emit_status_to_sid(request.sid, force=True)


@socketio.on("ros2-unsubscribe-request", namespace="/qlook")
def ros2_unsubscribe_request(json: Dict[str, Any]) -> None:
    logger.info(f"Got 'ros2-unsubscribe-request' from {request.sid}")
    topic_name = (json or {}).get("topic_name")
    field_name = (json or {}).get("field_name")
    role = (json or {}).get("role", "")
    options = (json or {}).get("options") or {}
    stream_key = (json or {}).get("stream_key")
    success = False
    if stream_key is not None:
        success = ClientManager(socketio).remove_subscription_by_stream_key(
            request.sid,
            stream_key,
        )
    elif topic_name is not None:
        success = ClientManager(socketio).remove_subscription(
            request.sid,
            topic_name,
            field_name=field_name,
            role=role,
            options=options,
        )
    socketio.emit(
        "ros2-unsubscribe",
        {"success": success},
        to=request.sid,
        namespace="/qlook",
    )
    emit_status_to_sid(request.sid, force=True)


@socketio.on("ros2-status-request", namespace="/qlook")
def ros2_status_request(json: Dict[str, Any]) -> None:
    logger.info(f"Got 'ros2-status-request' from {request.sid}")
    topics = (json or {}).get("topics") or []
    stream_keys = (json or {}).get("stream_keys") or []
    manager = ClientManager(socketio)
    streams = manager.get_stream_statuses()
    if topics:
        topic_set = set(topics)
        streams = [stream for stream in streams if stream["topic"] in topic_set]
    if stream_keys:
        stream_key_set = set(stream_keys)
        streams = [stream for stream in streams if stream["stream_key"] in stream_key_set]
    socketio.emit(
        "ros2-status",
        {
            "health": manager.get_health_status(),
            "streams": streams,
        },
        to=request.sid,
        namespace="/qlook",
    )



def main() -> None:
    import argparse

    p = argparse.ArgumentParser(description="Graphical console for NECST system.")
    p.add_argument(
        "-p", "--port", type=int, default=8080, help="Port to use (default: 8080)"
    )
    p.add_argument(
        "-i",
        "--interface",
        type=str,
        help="Network interface or IP address to use "
        "(default: randomly chosen from available local IPv4 interfaces)",
    )
    args = p.parse_args()
    host = get_ip_address(args.interface)

    rclpy.init()
    logger.info(
        "\033[1;4mTo detach this server:\n"
        " (when run with `-it` options) press Ctrl-P then Ctrl-Q\n"
        " (otherwise) re-run as a background process, by attaching `&` at the end "
        "of the command\033[0m"
    )
    try:
        socketio.run(app, host=host, port=args.port, allow_unsafe_werkzeug=True)
    except Exception as e:
        logger.debug(str(e))
    finally:
        ClientManager().destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
