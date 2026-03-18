"use strict"

import * as quickLook from "./quick-look.js"
import { Graph } from "./chart.js"

const socket = io("/qlook")
let currentRole = ""

function requestTopicList(role = "") {
    currentRole = role
    Graph("#chart", socket).clear()
    socket.emit("ros2-topic-list-request", { role })
}

function main() {
    $("#ros2-topic-list").click(() => requestTopicList(""))
    $("#total_power").click(() => requestTopicList("total_power"))
    $("#2d-plot").click(() => requestTopicList("2d-plot"))
    $("#sis_iv").click(() => requestTopicList("sis_iv"))

    socket.on("ros2-topic-list", msg => quickLook.updateTopicList(socket, msg))
    socket.on("ros2-topic-field", msg => quickLook.updateTopicField(socket, msg))
    socket.on("ros2-message", msg => {
        Graph("#chart", socket).push(msg.topic_name, msg.data, currentRole)
    })
}

$(document).ready(main)
