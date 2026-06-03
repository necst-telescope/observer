"use strict"

const MAX_AZEL_POINTS = 10000
const MAX_SIS_IV_POINTS = 5000

const defaultConfig = {
    type: 'line',
    data: { datasets: [] },
    options: {
        animation: false,
        scales: {
            x: {
                type: 'linear',
                ticks: {
                    minRotation: 10,
                    maxRotation: 10,
                    callback: function (value, idx, ticks) {
                        const graph = this?.chart?.$graphInstance
                        if (graph?.drawingArray || graph?.drawingTwoFields || graph?.drawingAzEl) {
                            return value
                        }
                        const isoString = new Date(value).toISOString()
                        if (idx === ticks.length - 1) { return isoString }
                        if (idx === 0) { return /[0-9-]*T(.*)Z/.exec(isoString)?.[1] || isoString }
                        return /[0-9-]*T(.*).000Z/.exec(isoString)?.[1] || isoString
                    },
                },
            },
            y: {},
        }
    }
}

class _Graph {

    updaterId
    chart
    subscriptions = new Map()
    drawingArray = null
    drawingTwoFields = null
    drawingAzEl = null
    activeRole = ""
    roleOptions = new Map()

    constructor(ctx, socket, config = {}) {
        this.config = $.extend(true, {}, defaultConfig, config)
        this.chart = new Chart(ctx, this.config)
        this.chart.$graphInstance = this
        this.updaterId = setInterval(() => this.#update(), 200)
        this.duration = 60
        this.socket = socket
    }

    setRole(role = "", options = null) {
        this.activeRole = role || ""
        if (options != null) {
            this.setRoleOptions(this.activeRole, options)
        }
    }

    setRoleOptions(role = "", options = {}) {
        this.roleOptions.set(role || "", this.normalizeOptions(options))
    }

    getRoleOptions(role = this.activeRole) {
        return { ...(this.roleOptions.get(role || "") || {}) }
    }

    updateCurrentRoleOptions(options = {}) {
        const role = this.activeRole || ""
        const normalized = this.normalizeOptions(options)
        this.setRoleOptions(role, normalized)
        const entries = Array.from(this.subscriptions.values()).filter((entry) => (entry.role || "") === role)
        for (let entry of entries) {
            const existingData = this.findDataset(entry.streamKey)?.data || []
            this.removeDataset(entry.topic, entry.field, entry.role, entry.options)
            this.addDataset(entry.topic, entry.field, entry.role, normalized)
            const dataset = this.findDataset(this.streamKey(entry.topic, entry.field, entry.role, normalized))
            if (dataset) {
                dataset.data = existingData.slice(-Math.min(existingData.length, 256))
            }
        }
    }

    normalizeRoleFields(role = this.activeRole, field = "") {
        const effectiveRole = role || ""
        return (["", "total_power", "spectrum_decimated"].includes(effectiveRole) && field) ? [field] : []
    }

    normalizeOptions(options = {}) {
        const normalized = {}
        for (let key of Object.keys(options || {}).sort()) {
            const value = options[key]
            if (value === undefined || value === null || value === "") {
                continue
            }
            normalized[key] = value
        }
        return normalized
    }

    streamKey(topic, field, role = this.activeRole, options = null) {
        const normalizedOptions = this.normalizeOptions(options == null ? this.getRoleOptions(role) : options)
        return JSON.stringify({
            fields: this.normalizeRoleFields(role, field),
            options: normalizedOptions,
            role: role || "",
            topic,
        })
    }

    datasetLabel(topic, field, role = this.activeRole, options = null) {
        const effectiveField = (["", "total_power", "spectrum_decimated"].includes(role || "")) ? field : (role === "2d-plot" ? "path" : (role === "sis_iv" ? "curve" : field))
        const suffix = role ? ` [${role}]` : ""
        const normalizedOptions = this.normalizeOptions(options == null ? this.getRoleOptions(role) : options)
        const optionParts = []
        if (role === "spectrum_decimated" && normalizedOptions.max_points) {
            optionParts.push(`max_points=${normalizedOptions.max_points}`)
        }
        if (["total_power", "spectrum_decimated"].includes(role || "")) {
            const start = normalizedOptions.channel_start
            const stop = normalizedOptions.channel_stop
            if (start !== undefined || stop !== undefined) {
                optionParts.push(`channels=${start ?? 0}:${stop ?? 'end'}`)
            }
        }
        const optionSuffix = optionParts.length ? ` (${optionParts.join(", ")})` : ""
        return `${topic}::${effectiveField}${suffix}${optionSuffix}`
    }

    findDataset(streamKey) {
        return this.config.data.datasets.find((elem) => elem.streamKey === streamKey)
    }

    describeStream(streamKey) {
        const entry = this.subscriptions.get(streamKey)
        if (!entry) {
            const dataset = this.findDataset(streamKey)
            if (dataset) {
                return {
                    topic: dataset.topic,
                    field: dataset.field,
                    role: dataset.role,
                    options: { ...(dataset.options || {}) },
                    label: dataset.label,
                }
            }
            return { label: streamKey }
        }
        return {
            topic: entry.topic,
            field: entry.field,
            role: entry.role,
            options: { ...(entry.options || {}) },
            label: this.datasetLabel(entry.topic, entry.field, entry.role, entry.options),
        }
    }

    getSubscriptionsSnapshot() {
        return {
            role: this.activeRole,
            roleOptions: Object.fromEntries(this.roleOptions.entries()),
            subscriptions: Array.from(this.subscriptions.values()).map((entry) => {
                return {
                    stream_key: entry.streamKey,
                    topic: entry.topic,
                    field: entry.field,
                    role: entry.role,
                    options: { ...(entry.options || {}) },
                }
            }),
        }
    }

    restoreSubscriptions(snapshot = null) {
        const state = snapshot || this.getSubscriptionsSnapshot()
        const roleOptions = state.roleOptions || {}
        for (let role of Object.keys(roleOptions)) {
            this.setRoleOptions(role, roleOptions[role])
        }
        this.setRole(state.role || this.activeRole)
        for (let entry of (state.subscriptions || [])) {
            this.socket.emit("ros2-subscribe-request", {
                topic_name: entry.topic,
                field_name: entry.field,
                role: entry.role || "",
                options: entry.options || {},
            })
        }
    }

    listStreamKeys() {
        return Array.from(this.subscriptions.keys())
    }

    addDataset(topic, field, role = this.activeRole, options = null) {
        const effectiveOptions = this.normalizeOptions(options == null ? this.getRoleOptions(role) : options)
        const streamKey = this.streamKey(topic, field, role, effectiveOptions)
        if (!this.subscriptions.has(streamKey)) {
            this.subscriptions.set(streamKey, {
                streamKey,
                topic,
                field,
                role: role || "",
                options: effectiveOptions,
            })
            this.config.data.datasets.push(
                {
                    label: this.datasetLabel(topic, field, role, effectiveOptions),
                    data: [],
                    fill: false,
                    streamKey,
                    topic,
                    field,
                    role,
                    options: effectiveOptions,
                }
            )
        }
        this.socket.emit("ros2-subscribe-request", {
            topic_name: topic,
            field_name: field,
            role: role || "",
            options: effectiveOptions,
        })
    }

    removeDataset(topic, field, role = this.activeRole, options = null, { keepVisualData = false } = {}) {
        const effectiveOptions = this.normalizeOptions(options == null ? this.getRoleOptions(role) : options)
        const streamKey = this.streamKey(topic, field, role, effectiveOptions)
        this.socket.emit("ros2-unsubscribe-request", {
            topic_name: topic,
            field_name: field,
            role: role || "",
            options: effectiveOptions,
            stream_key: streamKey,
        })
        this.subscriptions.delete(streamKey)

        const idx = this.config.data.datasets.findIndex(
            (elem) => elem.streamKey === streamKey
        )
        if (idx !== -1) {
            if (keepVisualData) {
                this.config.data.datasets[idx].streamKey = `stale:${streamKey}`
            } else {
                this.config.data.datasets.splice(idx, 1)
            }
        }
        if (this.config.data.datasets.length === 0) {
            this.drawingArray = null
            this.drawingTwoFields = null
            this.drawingAzEl = null
        }
    }

    toggleDataset(topic, field, role = this.activeRole, options = null) {
        const effectiveOptions = this.normalizeOptions(options == null ? this.getRoleOptions(role) : options)
        const streamKey = this.streamKey(topic, field, role, effectiveOptions)
        if (this.subscriptions.has(streamKey)) {
            this.removeDataset(topic, field, role, effectiveOptions)
        } else {
            this.addDataset(topic, field, role, effectiveOptions)
        }
    }

    #update() {
        const xScale = this.config.options.scales.x
        const yScale = this.config.options.scales.y
        if (this.drawingAzEl) {
            xScale.min = 0
            xScale.max = 360
            yScale.min = 0
            yScale.max = 90
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
            xScale.min = undefined
            xScale.max = undefined
            xScale.ticks.min = undefined
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

    push(message) {
        const streamKey = message.stream_key || this.streamKey(message.topic_name, "", message.role || "")
        const subscription = this.subscriptions.get(streamKey)
        if (!subscription) { return }
        const dataset = this.findDataset(streamKey)
        if (!dataset) { return }
        const data = message.data || {}
        const field = subscription.field
        if (!(field in data) && !["2d-plot", "sis_iv"].includes(subscription.role || "")) { return }
        const value = data[field]
        const isArray = Array.isArray(value)
        const effectiveRole = subscription.role || message.role || this.activeRole

        if (effectiveRole === "2d-plot") {
            this.drawingArray = false
            this.drawingTwoFields = false
            this.drawingAzEl = true
            const scales = this.config.options.scales
            if (("lon" in data) && ("lat" in data)) {
                dataset.data.push({ x: data["lon"], y: data["lat"] })
                while (dataset.data.length > MAX_AZEL_POINTS) { dataset.data.shift() }
            }
            scales.x.title.text = "Azimuth [deg]"
            scales.y.title.text = "Elevation [deg]"
        } else if (effectiveRole === "sis_iv") {
            this.drawingArray = false
            this.drawingTwoFields = true
            this.drawingAzEl = false
            const scales = this.config.options.scales
            if (("voltage" in data) && ("current" in data)) {
                dataset.data.push({ x: data["voltage"], y: data["current"] })
                while (dataset.data.length > MAX_SIS_IV_POINTS) { dataset.data.shift() }
            }
            scales.x.title.text = "Voltage [mV]"
            scales.y.title.text = "Current [uA]"
        } else if (isArray) {
            this.drawingArray = true
            this.drawingTwoFields = false
            this.drawingAzEl = false
            dataset.data.length = 0
            const xValues = Array.isArray(data.observer_x_values) ? data.observer_x_values : null
            if (xValues && xValues.length === value.length) {
                dataset.data.push(...value.map((y, i) => { return { x: xValues[i], y } }))
            } else {
                dataset.data.push(...value.map((y, i) => { return { x: i, y } }))
            }
        } else {
            this.drawingArray = false
            this.drawingTwoFields = false
            this.drawingAzEl = false
            try {
                const timeValue = (("time" in data) ? data.time : undefined)
                const sampleTime = Number(timeValue)
                const time = Number.isFinite(sampleTime) ? sampleTime * 1e3 : Date.now()
                dataset.data.push({ x: time, y: value })
                const xMin = this.config.options.scales.x.min
                while ((dataset.data.length > 0) && (dataset.data[0].x < xMin)) { dataset.data.shift() }
            } catch (error) {
                console.debug(error)
            }
        }
    }

    destroy() {
        this.clear()
        clearInterval(this.updaterId)
        this.chart.destroy()
    }

    clear() {
        for (let entry of Array.from(this.subscriptions.values())) {
            this.removeDataset(entry.topic, entry.field, entry.role, entry.options)
        }
        this.config.data.datasets = []
    }
}

const GraphInstances = new Map()

const Graph = (id, socket, config) => {
    if (!GraphInstances.has(id)) {
        const _ctx = $(`#${id.replace(/^#/, '')}`)
        const ctx = _ctx[0].getContext("2d")
        GraphInstances.set(id, new _Graph(ctx, socket, config))
    } else if (socket) {
        GraphInstances.get(id).socket = socket
    }
    return GraphInstances.get(id)
}

export { Graph }
