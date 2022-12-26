"use strict"

import * as quickLook from "./quick-look.js"
import { Graph } from "./chart.js"



function main() {
    const socket = io("/qlook")
    $("#ros2-topic-list").click(
        () => {
            Graph("#chart", socket).clear()
            socket.emit("ros2-topic-list-request", {})
        }
    )

    socket.on("ros2-topic-list", msg => quickLook.updateTopicList(socket, msg))
    socket.on("ros2-topic-field", msg => quickLook.updateTopicField(socket, msg))
    socket.on("ros2-message", msg => {
        Graph("#chart", socket).push(msg.topic_name, msg.data)
    })
}

$(document).ready(main)
