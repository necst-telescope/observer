"use strict"

import { parseDataType } from "./ros-tools.js"
import { toMap } from "./utils.js"
import { Graph } from "./chart.js"


function updateTopicList(socket, msg = {topic_split: {}}) {
    $("#message-fields").empty()
    const container = $("#category-list")
    container.empty()
    const container2 = $("#topic-list")
    container2.empty()
    const topic_list = Object.keys(msg.topic_split)
    const topics = toMap(topic_list)
    if (!topics.size) { return }
    const cat = {}
    for (let topic of topics.values()) {
        const [blanck, cat_name, ..._topic_name] = topic.split("/")
        const topic_name = _topic_name.join("/")
        if (cat_name in cat){
            cat[cat_name].push(topic_name)
        } else {
            cat[cat_name] = [topic_name]
        }
    }
    for (let cat_name of toMap(Object.keys(cat))) {
        const text = $("<code>").text(cat_name)
        $("<button>").html(text).appendTo(container).click(
            () => {
                for (let name of cat[cat_name]) {
                    const text_ = $("<code>").text(name)
                    $("<button>").html(text_).appendTo(container2).click(
                        () =>{
                        const topic = [blanck, cat_name, name].join("/")
                        socket.emit("ros2-topic-field-request", 
                        { topic_name: [msg.topic_split[topic].system, msg.topic_split[topic].observatory, topic]})
                        }
                    )
                }
            }
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
