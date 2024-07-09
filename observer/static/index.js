"use strict"

import * as quickLook from "./quick-look.js"
import { Graph } from "./chart.js"



function main() {
    const socket = io("/qlook")
    const role = ""
    $("#ros2-topic-list").click(
        () => {
            Graph("#chart", socket).clear()
            socket.emit("ros2-topic-list-request", { "role": role })
        }
    )

    socket.on("ros2-topic-list", msg => quickLook.updateTopicList(socket, msg))
    socket.on("ros2-topic-field", msg => quickLook.updateTopicField(socket, msg))
    socket.on("ros2-message", msg => {
        Graph("#chart", socket).push(msg.topic_name, msg.data)
    })
}

function Sub() {
    const socket = io("/qlook")
    const role = "total_power"
    $("#total_power").click(
        () => {
            socket.emit("ros2-topic-list-request", { "role": role })
        }
    )

    socket.on("ros2-topic-list", msg => quickLook.updateTopicList(socket, msg))
    socket.on("ros2-topic-field", msg => quickLook.updateTopicField(socket, msg))
    socket.on("ros2-message", msg => {
        Graph("#chart", socket).push(msg.topic_name, msg.data, role)
    })
}

function sub2() {
    const socket = io("/qlook")
    const role = "2d-plot"
    $("#2d-plot").click(
        () => {
            // TODO: Update chart in 2D-plot mode from 1D-plot mode.
            socket.emit("ros2-topic-list-request", { "role": role })
        }
    )

    socket.on("ros2-topic-list", msg => quickLook.updateTopicList(socket, msg))
    socket.on("ros2-topic-field", msg => quickLook.updateTopicField(socket, msg))
    socket.on("ros2-message", msg => {
        Graph("#chart", socket).push(msg.topic_name, msg.data, role)
    })
}

$(document).ready(main)
$(document).ready(Sub)
$(document).ready(sub2)
