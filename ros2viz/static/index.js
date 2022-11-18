function updateTopicListSelect(sock, topicList) {
    const container = $("#select-topic")
    container.empty()
    for (let topic of topicList) {
        const text = $("<code>").text(topic)
        $("<button>").html(text).appendTo(container).click(
            () => sock.emit("ros2-topic-field-request", { topic_name: topic })
        )
    }
}

function updateTopicFieldList(sock, fields, topicName) {
    const container = $("#select-field")
    container.empty()
    for (let name in fields) {
        const type = fields[name]
        const text = $("<code>").text(name)
        const drawable = type.startsWith("int")
            || type.startsWith("uint")
            || type.startsWith("float")
            || type.startsWith("double")
        $("<button>").html(text).appendTo(container).click(
            { io: sock, topicName: topicName, fieldName: name }, drawChart
        ).prop("disabled", !drawable)
    }
}

function drawChart(event) {
    const sock = event.data.io
    const topicName = event.data.topicName
    const fieldName = event.data.fieldName

    sock.emit("ros2-message-request", { topic_name: topicName })

    const defaultData = [{ x: Date.now(), y: null }]
    const config = {
        type: "line",
        data: {
            datasets: [{
                label: fieldName,
                backgroundColor: "rgb(255, 99, 132)",
                borderColor: "rgb(255, 99, 132)",
                data: defaultData,
                fill: false
            }]
        },
        options: {
            responsive: true,
            title: { display: true, text: topicName },
            tooltips: { mode: "index", intersect: false },
            hover: { mode: "nearest", intersect: true },
            scales: {
                xAxes: [{
                    type: "linear",
                    title: { display: true, text: "Time" },
                    min: Date.now() - 30 * 1e3,
                    max: Date.now(),
                    ticks: {
                        display: true,
                        min: Date.now() - 30 * 1e3,
                        max: Date.now(),
                        userCallback: (t, i) => new Date(t)
                    }
                }],
                yAxes: [{ title: { display: true, text: "Value" } }],
            }
        }
    }
    const ctx = $("<canvas>").appendTo($("#charts"))[0].getContext("2d")
    const lineChart = new Chart(ctx, config)
    sock.on("ros-message", msg => {
        const dataset = config.data.datasets[0]
        if (msg.topic !== topicName) { return }
        const now = Date.now()
        dataset.data.push({ x: now, y: msg.msg[fieldName] })
        config.options.scales.xAxes[0].min = now - 30 * 1e3
        config.options.scales.xAxes[0].max = now
        config.options.scales.xAxes[0].ticks.min = now - 30 * 1e3
        config.options.scales.xAxes[0].ticks.max = now
        if (dataset.data.length > 0) {
            while (dataset.data[0].x < config.options.scales.xAxes[0].min) {
                _ = dataset.data.shift()
            }
        }
        lineChart.update()
    })
}

function main() {
    const socket = io()

    $("#ros2-topic-list").click(
        () => socket.emit("ros2-topic-list-request", {})
    )
    socket.on("ros2-topic-list", msg => {
        updateTopicListSelect(socket, msg.topic_names)
    })
    socket.on("ros2-topic-field", msg => {
        updateTopicFieldList(socket, msg.fields, msg.topic_name)
    })
}

$(document).ready(main)
