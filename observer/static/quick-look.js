"use strict"

import { parseDataType } from "./ros-tools.js"
import { Graph } from "./chart.js"

function updateTopicList(socket, msg = {topic_split: {}}) {
    const container = $("#category-list")
    container.empty()
    const container2 = $("#topic-list")
    container2.empty()
    const container3 = $("#message-fields")
    container3.empty()
    const topic_entries = Object.entries(msg.topic_split)
    const topics = new Map(topic_entries)
    if (!topics.size) { return }
    const cat = {}
    for (let [fullTopic, meta] of topics.entries()) {
        const displayTopic = meta.display_topic || meta.topic || fullTopic
        const [, cat_name, ..._topic_name] = displayTopic.split("/")
        const topic_name = _topic_name.join("/")
        if (cat_name in cat) {
            cat[cat_name].push({label: topic_name, fullTopic})
        } else {
            cat[cat_name] = [{label: topic_name, fullTopic}]
        }
    }
    for (let cat_name of Object.keys(cat)) {
        const text = $("<code>").text(cat_name)
        $("<button>").html(text).appendTo(container).click(
            () => {
                container2.empty()
                container3.empty()
                for (let entry of cat[cat_name]) {
                    const text_ = $("<code>").text(entry.label)
                    $("<button>").html(text_).appendTo(container2).click(
                        () => {
                            const meta = msg.topic_split[entry.fullTopic]
                            if (!meta) {
                                console.warn(`Topic info not found for ${entry.fullTopic}`)
                                return
                            }
                            const topic = meta.topic || entry.fullTopic
                            socket.emit("ros2-topic-field-request",
                                { topic_name: [meta.system, meta.observatory, topic] })
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
    if (msg.error) {
        console.warn(`Server error: ${msg.error}`)
        $("<code>").text(`Server error: ${msg.error}`).appendTo(container)
        return
    }
    const fields = new Map(Object.entries(msg.fields || {}))
    if (!fields.size) {
        console.warn("Message for the topic has no field info.")
        $("<code>").text("Message for the topic has no field info.").appendTo(container)
        return
    }
    const activeRole = Graph("#chart", socket).activeRole || ""
    if (["2d-plot", "sis_iv"].includes(activeRole)) {
        const label = (activeRole === "2d-plot") ? "plot" : "curve"
        const requiredFields = (activeRole === "2d-plot") ? ["lon", "lat"] : ["voltage", "current"]
        const enabled = requiredFields.every(name => fields.has(name))
        const text = $("<code>").text(label)
        $("<button>")
            .html(text)
            .appendTo(container)
            .prop("disabled", !enabled)
            .attr("title", enabled ? "" : `requires fields: ${requiredFields.join(", ")}`)
            .click(() => {
                if (!enabled) { return }
                Graph("#chart", socket).toggleDataset(msg.topic_name, "", activeRole)
            })
        return
    }
    for (let [name, type] of fields) {
        const text = $("<code>").text(name)
        const dataKind = parseDataType(type)
        const requiresArray = ["total_power", "spectrum_decimated"].includes(activeRole)
        const enabled = requiresArray ? (dataKind.numerical && dataKind.array) : dataKind.numerical
        $("<button>")
            .html(text)
            .appendTo(container)
            .prop("disabled", !enabled)
            .data(dataKind)
            .click(
                () => {
                    Graph("#chart", socket).toggleDataset(msg.topic_name, name)
                }
            )
    }
}

export { updateTopicList, updateTopicField }
