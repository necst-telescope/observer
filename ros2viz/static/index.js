"use strict"

import { parseDataType } from "./ros-tools.js"
import { toMap } from "./utils.js"
import { Graph } from "./chart.js"


function updateTopicList(socket, msg = { topic_names: [] }) {
    $("#message-fields").empty()
    const container = $("#topic-list")
    container.empty()
    const topics = toMap(msg.topic_names)
    if (!topics.size) { return }
    for (let topic of topics.values()) {
        const text = $("<code>").text(topic)
        $("<button>").html(text).appendTo(container).click(
            () => socket.emit("ros2-topic-field-request", { topic_name: topic })
        )
    }
}

function updateTopicField(socket, msg = { topic_name: "", fields: [], error: "" }) {
    const container = $("#message-fields")
    container.empty()
    if (msg.error) { throw `Server error: ${msg.error}` }
    const fields = toMap(msg.fields)
    if (!fields.size) { throw "Message for the topic has no field info." }
    for (let [name, type] of fields) {
        const text = $("<code>").text(name)
        const dataKind = parseDataType(type)
        $("<button>")
            .html(text)
            .appendTo(container)
            .prop("disabled", !dataKind.numerical)
            .data(dataKind)
            .click(
                () => {
                    Graph("#chart", socket).toggleDataset(msg.topic_name, name)
                }
            )
    }
}

function main() {
    const socket = io()
    $("#ros2-topic-list").click(
        () => {
            Graph("#chart", socket).clear()
            socket.emit("ros2-topic-list-request", {})
        }
    )

    socket.on("ros2-topic-list", msg => updateTopicList(socket, msg))
    socket.on("ros2-topic-field", msg => updateTopicField(socket, msg))
    socket.on("ros2-message", msg => {
        Graph("#chart", socket).push(msg.topic_name, msg.data)
    })
}

$(document).ready(main)
