import logging
import os
import time
import traceback
from pathlib import Path
from typing import Dict
import re

import neclib
import rclpy
import tomlkit
from flask import Flask, Response, escape, redirect, render_template, request, url_for
from flask_socketio import SocketIO, join_room, leave_room
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


@app.route("/")
def index() -> Response:
    return render_template("index.html")


@app.route("/qlook")
def qlook() -> str:
    return render_template("qlook/index.html")


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
def connect(auth) -> bool:
    logger.info(f"New Connection: {request.sid}")
    success = ClientManager(socketio).add_client(request.sid)
    return success


@socketio.on("disconnect", namespace="/qlook")
def disconnect() -> bool:
    logger.info(f"Disconnected: {request.sid}")
    for _ in range(10):
        if ClientManager(socketio).remove_client(request.sid):
            break
        time.sleep(0.1)
    return True


@socketio.on("ros2-topic-list-request", namespace="/qlook")
def ros2_topic_list_request(json: Dict[str, str]) -> None:
    logger.info(f"Got 'ros2-topic-list-request' from {request.sid}")
    topics = ClientManager(socketio).get_topic_names_and_types()
    if json["role"] == "total_power":
        topics = [topic for topic in topics if "quick_spectra" in topic[0]]
        if not topics:
            logger.info("There is no spectra data in ROS topics.")
    if json["role"] == "2d-plot":
        topics = [
            topic for topic in topics if re.search("/encoder", topic[0])
        ]
        if not topics:
            logger.info("There is no data that can be plotted in 2-D in ROS topics.")
    if json["role"] == "sis_iv":
        topics = [
            topic for topic in topics if re.search("/sis_bias", topic[0])
        ]
        if not topics:
            logger.info("There is no data that can be plotted in 2-D in ROS topics.")
    topic_split = {}
    for l in topics:
        sp = re.split(r"(?=/)", l[0], 3)
        if len(sp) != 4:
            topic_split[sp[1]] = {"system": "", "observatory": ""}
        else:
            topic_split[sp[3]] = {"system": sp[1], "observatory": sp[2]}
    socketio.emit(
        "ros2-topic-list",
        {"topic_split": topic_split},
        to=request.sid,
        namespace="/qlook",
    )


@socketio.on("ros2-topic-field-request", namespace="/qlook")
def ros2_topic_field_request(json: Dict[str, str]) -> None:
    logger.info(f"Got 'ros2-topic-field-request' from {request.sid}")
    topic_info = json["topic_name"]
    topic_name = "".join(topic_info)
    msg_type = get_msg_type(topic_name)
    if msg_type is None:
        socketio.emit(
            "ros2-topic-field",
            {"error": f"Cannot find message type for {topic_name!r}"},
            to=request.sid,
            namespace="/qlook",
        )
    socketio.emit(
        "ros2-topic-field",
        {"topic_name": topic_name, "fields": msg_type.get_fields_and_field_types()},
        to=request.sid,
        namespace="/qlook",
    )


@socketio.on("ros2-subscribe-request", namespace="/qlook")
def ros2_subscribe_request(json: Dict[str, str]) -> None:
    logger.info(f"Got 'ros2-subscribe-request' from {request.sid}")
    topic_name = json["topic_name"]
    success = ClientManager(socketio).add_subscription(request.sid, topic_name)
    if success:
        join_room(topic_name)
        logger.info(f"{request.sid} joined the room {topic_name!r}")
    socketio.emit("ros2-subscribe", {"success": success}, to=request.sid)


@socketio.on("ros2-unsubscribe-request", namespace="/qlook")
def ros2_unsubscribe_request(json: Dict[str, str]) -> None:
    logger.info(f"Got 'ros2-unsubscribe-request' from {request.sid}")
    topic_name = json["topic_name"]
    success = ClientManager(socketio).remove_subscription(request.sid, topic_name)
    if success:
        leave_room(topic_name)
        logger.info(f"{request.sid} left the room {topic_name!r}")
    socketio.emit("ros2-unsubscribe", {"success": success}, to=request.sid)


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
