"use strict"

import { DefaultMap } from "./utils.js"

class _Graph {
    constructor(ctx, socket, config = { title: "", xLabel: "Time", yLabel: "Value" }) {
        this.duration = 30
        this.config = {
            type: "line",
            data: {
                datasets: []
            },
            options: {
                responsive: true,
                plugins: {
                    title: { display: true, text: config.title }
                },
                tooltips: { mode: "nearest", intersect: false },
                hover: { mode: "index", intersect: true },
                animation: { duration: 0 },
                scales: {
                    x: {
                        type: "linear",
                        title: { display: true, text: config.xLabel },
                        min: Date.now() - this.duration * 1e3,
                        max: Date.now(),
                        ticks: {
                            minRotation: 10,
                            maxRotation: 10,
                            callback: (value, idx, ticks) => {
                                if (this.drawingArray || this.drawingTwoFields || this.drawingAzEl) { return value }
                                const isoString = new Date(value).toISOString()
                                if (idx === ticks.length - 1) { return isoString }
                                if (idx === 0) { return /[0-9-]*T(.*)Z/.exec(isoString)[1] }
                                return /[0-9-]*T(.*).000Z/.exec(isoString)[1]
                            }
                        }
                    },
                    y: {
                        title: { display: true, text: config.yLabel }
                    }
                }
            }
        }
        this.ctx = ctx
        this.chart = new Chart(this.ctx, this.config)
        this.drawingArray = null
        this.drawingTwoFields = null
        this.drawingAzEl = null
        this.updaterId = setInterval(this.#update.bind(this), 100)
        // Without `bind`, `this` inside `this.#update` will point to `setInterval`'s.

        this.subscriptions = new DefaultMap(() => [])
        this.socket = socket
    }

    addDataset(topic, field) {
        const subs = this.subscriptions.get(topic)
        if (subs.length === 0) {
            this.socket.emit("ros2-subscribe-request", { topic_name: topic })
        }
        if (!subs.includes(field)) { subs.push(field) }

        this.config.data.datasets.push(
            { label: this.#id(topic, field), data: [], fill: false }
        )
    }

    removeDataset(topic, field) {
        const subs = this.subscriptions.get(topic)
        if (subs.includes(field)) {
            const idx = subs.indexOf(field)
            subs.splice(idx, 1)
        }
        if (subs.length === 0) {
            this.socket.emit("ros2-unsubscribe-request", { topic_name: topic })
            this.subscriptions.delete(topic)
        }

        const idx = this.config.data.datasets.findIndex(
            (elem) => elem.label === this.#id(topic, field)
        )
        this.config.data.datasets.splice(idx, 1)
        if (this.config.data.datasets.length === 0) { this.drawingArray = null }
    }

    toggleDataset(topic, field) {
        const idx = this.config.data.datasets.findIndex(
            (elem) => elem.label === this.#id(topic, field)
        )
        if (idx !== -1) {
            this.removeDataset(topic, field)
        } else {
            this.addDataset(topic, field)
        }
    }

    #id(topic, field) { return `${topic}::${field}` }

    #update() {
        const xScale = this.config.options.scales.x
        const yScale = this.config.options.scales.y
        if (this.drawingAzEl) {
            xScale.min = 0
            xScale.max = 360
            yScale.min = 0
            yScale.max = 90 // TODO: change plot range what user want to plot
            xScale.ticks.min = undefined
            xScale.ticks.max = undefined
            yScale.ticks.min = undefined
            yScale.ticks.max = undefined
            this.chart.update()
        } 
        else if (this.drawingTwoFields) {
            xScale.min = undefined
            xScale.max = undefined
            yScale.min = undefined
            yScale.max = undefined
            xScale.ticks.min = undefined
            xScale.ticks.max = undefined
            yScale.ticks.min = undefined
            yScale.ticks.max = undefined
            this.chart.update()
        } else if (this.drawingArray) {
            xScale.min = 0
            xScale.max = undefined
            xScale.ticks.min = 0
            xScale.ticks.max = undefined
            this.chart.update()
        } else {
            const now = Date.now()
            xScale.min = now - this.duration * 1e3
            xScale.max = now
            xScale.ticks.min = now - this.duration * 1e3
            xScale.ticks.max = now
            this.chart.update()
        }
    }

    push(topic, data, role = "None") {
        for (let field of this.subscriptions.get(topic)) {
            const idx = this.config.data.datasets.findIndex(
                (elem) => elem.label === this.#id(topic, field)
            )
            const dataset = this.config.data.datasets[idx]
            const isArray = data[field].length > 1  // undefined > 1 --> false
            if (role === "2d-plot") {
                // HACK: Consider that unite two arguments into one.
                this.drawingArray = false
                this.drawingTwoFields = false
                this.drawingAzEl = true
                const scales = this.config.options.scales
                dataset.data.push({ x: data["lon"], y: data["lat"] })
                scales.x.title.text = "Azimuth [deg]"
                scales.y.title.text = "Elevation [deg]"
            } else if (role == "sis_iv") {
                this.drawingArray = false
                this.drawingTwoFields = true
                this.drawingAzEl = false
                const scales = this.config.options.scales
                dataset.data.push({ x: data["voltage"], y: data["current"] })
                scales.x.title.text = "Voltage [mV]"
                scales.y.title.text = "Current [uA]"
            } else if (isArray) {
                if (role === "total_power") {
                    const total_power = data[field].reduce((accumulator, currentValue) => accumulator + currentValue, 0)
                    this.drawingArray = false
                    this.drawingTwoFields = false
                    this.drawingAzEl = false
                    try {
                        const time = data.time * 1e3 || Date.now()
                        dataset.data.push({ x: time, y: total_power })
                        const xMin = this.config.options.scales.x.min
                        while (dataset.data[0].x < xMin) { dataset.data.shift() }
                    } catch (error) {
                        console.debug(error)
                    }
                } else {
                    this.drawingArray = true
                    this.drawingTwoFields = false
                    this.drawingAzEl = false
                    dataset.data.length = 0
                    dataset.data.push(...data[field].map((x, i) => { return { x: i, y: x } }))
                }
            } else {
                this.drawingArray = false
                this.drawingTwoFields = false
                this.drawingAzEl = false
                try {
                    const time = data.time * 1e3 || Date.now()
                    dataset.data.push({ x: time, y: data[field] })
                    const xMin = this.config.options.scales.x.min
                    while (dataset.data[0].x < xMin) { dataset.data.shift() }
                } catch (error) {
                    // Catch and ignore array length change during appending the data
                    // This is caused by non-blocking `removeDataset`
                    console.debug(error)
                }
            }
        }
    }

    destroy() {
        this.clear()
        clearInterval(this.updaterId)
        this.chart.destroy()
    }

    clear() {
        for (let [topic, fields] of this.subscriptions) {
            for (let field of fields) {
                this.removeDataset(topic, field)
            }
        }
    }
}

const GraphInstances = new Map()

const Graph = (id, socket, config) => {
    if (!GraphInstances.has(id)) {
        const _ctx = $(`#${id.replace(/^#/, '')}`)
        const ctx = _ctx[0].getContext("2d")
        GraphInstances.set(id, new _Graph(ctx, socket, config))
    }
    return GraphInstances.get(id)
}

export { Graph }
