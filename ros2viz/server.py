import logging

from flask import Flask, render_template
from flask_socketio import SocketIO, emit

from .address import get_ip_address
from .ros_context import ros2env, ros2node
from .ros_topic import SubscribeTo, get_msg_type

logger = logging.getLogger(__name__)

app = Flask(__name__)
socketio = SocketIO(app, cors_allowed_origins="*")


@app.route("/")
def index() -> str:
    return render_template("index.html")


@socketio.on("connect")
def connect():
    pass


@socketio.on("disconnect")
def disconnect():
    pass


@socketio.on("ros2-topic-list-request")
def ros2_topic_list_request(json):
    node = ros2node()
    topics = node.get_topic_names_and_types()
    topic_names = [t[0] for t in topics]
    emit("ros2-topic-list", {"topic_names": topic_names})


@socketio.on("ros2-topic-field-request")
def ros2_topic_field_request(json):
    topic_name = json["topic_name"]
    msg_type = get_msg_type(topic_name)
    if msg_type is None:
        return
    emit(
        "ros2-topic-field",
        {"topic_name": topic_name, "fields": msg_type.get_fields_and_field_types()},
    )


@socketio.on("ros2-message-request")
def ros2_message_request(json):
    topic_name = json["topic_name"]
    sub = SubscribeTo(topic_name, socketio)  # TODO: manage this
    sub.start()


def main():
    with ros2env():
        socketio.run(app, host=get_ip_address())
