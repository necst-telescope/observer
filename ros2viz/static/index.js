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

    const config = {
        type: "line",
        data: {
            datasets: [{
                label: fieldName,
                backgroundColor: "rgb(255, 99, 132)",
                borderColor: "rgb(255, 99, 132)",
                data: [],
                fill: false
            }]
        },
        options: {
            responsive: true,
            plugins: {
                title: { display: true, text: topicName }
            },
            tooltips: { mode: "nearest", intersect: false },
            hover: { mode: "index", intersect: true },
            animation: { duration: 50 },
            scales: {
                x: {
                    type: "linear",
                    title: { display: true, text: "Time" },
                    min: Date.now() - 30 * 1e3,
                    max: Date.now(),
                    ticks: {
                        display: true,
                        min: Date.now() - 30 * 1e3,
                        max: Date.now(),
                        callback: (value, idx, ticks) => new Date(value).toISOString()
                    }
                },
                y: { title: { display: true, text: "Value" } },
            }
        }
    }
    const ctx = $("#time-series")[0].getContext("2d")
    const lineChart = new Chart(ctx, config)
    sock.on("ros-message", msg => {
        const dataset = config.data.datasets[0]
        if (msg.topic !== topicName) { return }
        const now = Date.now()
        dataset.data.push({ x: now, y: msg.msg[fieldName] })
        while (true) {
            if (dataset.data.length > 0
                && dataset.data[0].x < config.options.scales.x.min
            ) { _ = dataset.data.shift() } else { break }
        }
    })
    const update = setInterval(() => {
        const now = Date.now()
        config.options.scales.x.min = now - 30 * 1e3
        config.options.scales.x.max = now
        config.options.scales.x.ticks.min = now - 30 * 1e3
        config.options.scales.x.ticks.max = now
        lineChart.update()
    }, 100)
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
