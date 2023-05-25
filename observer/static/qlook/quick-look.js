"use strict"

import { parseDataType } from "../ros-tools.js"
import { toMap } from "../utils.js"
import { Graph } from "../chart.js"


function updateTopicList(socket, msg = {topic_split: {}}) {
    $("#message-fields").empty()
    const container = $("#topic-list")
    container.empty()
    const topic_list = Object.keys(msg.topic_split)
    const topics = toMap(topic_list)
    if (!topics.size) { return }
    for (let topic of topics.values()) {
        const text = $("<code>").text(topic)
        $("<button>").html(text).appendTo(container).click(
            () => socket.emit("ros2-topic-field-request", 
            { topic_name: [msg.topic_split[topic].system, msg.topic_split[topic].observatory, topic]})
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

export { updateTopicList, updateTopicField }
