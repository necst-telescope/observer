"use strict"

import * as quickLook from "./quick-look.js"
import { Graph } from "./chart.js"

const socket = io("/qlook", { autoConnect: false })
let currentRole = ""
let statusPollId = null
let statusRefreshId = null
const streamStatusState = new Map()
const streamStatusHistory = new Map()
const STREAM_HISTORY_LIMIT = 20

function graph() {
    return Graph("#chart", socket)
}

function setConnectionStatus(text, isConnected = false) {
    const elem = $("#connection-status")
    elem.text(text)
    elem.toggleClass("connected", isConnected)
    elem.toggleClass("disconnected", !isConnected)
}

function setObserverStatus(text) {
    $("#observer-status").text(text)
}

function parseOptionalIntegerInput(selector) {
    const rawText = `${$(selector).val() ?? ""}`.trim()
    if (!rawText.length) {
        return undefined
    }
    const value = Number.parseInt(rawText, 10)
    return Number.isFinite(value) ? value : undefined
}

function roleOptionsFromControls(role = currentRole) {
    if (role === "spectrum_decimated") {
        const raw = Number.parseInt($("#max-points-input").val(), 10)
        const maxPoints = Number.isFinite(raw) ? raw : 1024
        const options = { max_points: maxPoints }
        const channelStart = parseOptionalIntegerInput("#decimation-channel-start-input")
        const channelStop = parseOptionalIntegerInput("#decimation-channel-stop-input")
        if (channelStart !== undefined) {
            options.channel_start = channelStart
        }
        if (channelStop !== undefined) {
            options.channel_stop = channelStop
        }
        return options
    }
    if (role === "total_power") {
        const options = {}
        const channelStart = parseOptionalIntegerInput("#channel-start-input")
        const channelStop = parseOptionalIntegerInput("#channel-stop-input")
        if (channelStart !== undefined) {
            options.channel_start = channelStart
        }
        if (channelStop !== undefined) {
            options.channel_stop = channelStop
        }
        return options
    }
    return {}
}

function syncControlsFromRole(role = currentRole) {
    const options = graph().getRoleOptions(role)
    const currentValue = options.max_points || 1024
    $("#max-points-input").val(currentValue)
    $("#decimation-channel-start-input").val(options.channel_start ?? "")
    $("#decimation-channel-stop-input").val(options.channel_stop ?? "")
    $("#channel-start-input").val(options.channel_start ?? "")
    $("#channel-stop-input").val(options.channel_stop ?? "")
    const showDecimation = role === "spectrum_decimated"
    const showTotalPower = role === "total_power"
    $("#decimation-controls").toggleClass("hidden", !showDecimation)
    $("#total-power-controls").toggleClass("hidden", !showTotalPower)
}

function formatAgeFromStatus(stream, key) {
    const timestamp = stream?.[key]
    if (timestamp == null) {
        return "-"
    }
    const serverTime = Number(stream?.server_time)
    const receivedAtMs = Number(stream?._received_at_ms)
    let age = NaN
    if (Number.isFinite(serverTime)) {
        const baseAge = serverTime - Number(timestamp)
        if (Number.isFinite(baseAge)) {
            const extraAge = Number.isFinite(receivedAtMs) ? ((Date.now() - receivedAtMs) / 1000.0) : 0.0
            age = baseAge + extraAge
        }
    }
    if (!Number.isFinite(age)) {
        age = Date.now() / 1000.0 - Number(timestamp)
    }
    return Number.isFinite(age) ? `${Math.max(0, age).toFixed(1)}s` : "-"
}

function summarizeHistory(streamKey) {
    const history = streamStatusHistory.get(streamKey) || []
    if (!history.length) {
        return "no recent transitions"
    }
    return history.slice().reverse().map(item => `${item.at_text}: ${item.from} -> ${item.to}`).join("\n")
}

function badgeTextForStream(stream) {
    const desc = graph().describeStream(stream.stream_key)
    const emitAge = formatAgeFromStatus(stream, "last_emit_time")
    const rxAge = formatAgeFromStatus(stream, "last_rx_time")
    return `${desc.label} | ${stream.status} | emit_age=${emitAge} | rx_age=${rxAge} | throttled=${stream.throttle_count}`
}

function tooltipTextForStream(stream) {
    const desc = graph().describeStream(stream.stream_key)
    const lines = [
        desc.label,
        `status=${stream.status}`,
        `last_emit_age=${formatAgeFromStatus(stream, "last_emit_time")}`,
        `last_rx_age=${formatAgeFromStatus(stream, "last_rx_time")}`,
        `last_throttle_age=${formatAgeFromStatus(stream, "last_throttle_time")}`,
        `emit_count=${stream.emit_count ?? 0}`,
        `rx_count=${stream.rx_count ?? 0}`,
        `throttle_count=${stream.throttle_count ?? 0}`,
        `client_count=${stream.client_count ?? 0}`,
        "history:",
        summarizeHistory(stream.stream_key),
    ]
    return lines.join("\n")
}

function updateStreamStatusHistories(streams = []) {
    const activeKeys = new Set(streams.map(stream => stream.stream_key))
    const currentSubscriptions = new Set(graph().listStreamKeys())
    for (let [streamKey, previous] of Array.from(streamStatusState.entries())) {
        if (!activeKeys.has(streamKey)) {
            streamStatusState.delete(streamKey)
        }
    }
    for (let streamKey of Array.from(streamStatusHistory.keys())) {
        if (!activeKeys.has(streamKey) && !currentSubscriptions.has(streamKey)) {
            streamStatusHistory.delete(streamKey)
        }
    }
    for (let stream of streams) {
        const previous = streamStatusState.get(stream.stream_key)
        if (previous && previous.status !== stream.status) {
            const history = streamStatusHistory.get(stream.stream_key) || []
            history.push({
                at: Date.now(),
                at_text: new Date().toLocaleTimeString(),
                from: previous.status,
                to: stream.status,
            })
            while (history.length > STREAM_HISTORY_LIMIT) {
                history.shift()
            }
            streamStatusHistory.set(stream.stream_key, history)
        } else if (!previous && !streamStatusHistory.has(stream.stream_key)) {
            streamStatusHistory.set(stream.stream_key, [])
        }
        streamStatusState.set(stream.stream_key, { ...stream, _received_at_ms: Date.now() })
    }
}

function renderStreamStatuses(streams = null) {
    const container = $("#stream-status-list")
    const panel = $("#stream-status-panel")
    const stateStreams = streams || Array.from(streamStatusState.values())
    if (streams) {
        updateStreamStatusHistories(streams)
    }
    container.empty()
    if (!stateStreams.length) {
        panel.removeAttr("open")
        return
    }
    for (let stream of stateStreams) {
        $("<code>")
            .addClass(`status-chip ${stream.status || 'idle'}`)
            .attr("title", tooltipTextForStream(stream))
            .text(badgeTextForStream(stream))
            .appendTo(container)
    }
}

function requestTopicList(role = "", { clearGraph = true } = {}) {
    currentRole = role
    const options = roleOptionsFromControls(role)
    graph().setRole(role, options)
    syncControlsFromRole(role)
    if (clearGraph) {
        graph().clear()
    }
    socket.emit("ros2-topic-list-request", { role })
}

function pollObserverStatus() {
    socket.emit("ros2-status-request", {
        stream_keys: graph().listStreamKeys(),
    })
}

function startStatusPolling() {
    if (statusPollId !== null) {
        clearInterval(statusPollId)
    }
    statusPollId = setInterval(() => {
        if (socket.connected) {
            pollObserverStatus()
        }
    }, 30000)
    if (statusRefreshId !== null) {
        clearInterval(statusRefreshId)
    }
    statusRefreshId = setInterval(() => {
        if (streamStatusState.size > 0) {
            renderStreamStatuses()
        }
    }, 1000)
}

function restoreStateAfterReconnect() {
    requestTopicList(currentRole, { clearGraph: false })
    graph().restoreSubscriptions()
    pollObserverStatus()
}

function applyRoleOptions() {
    const role = currentRole || graph().activeRole || ""
    const options = roleOptionsFromControls(role)
    graph().updateCurrentRoleOptions(options)
    syncControlsFromRole(role)
    pollObserverStatus()
}

function bindSocketEvents() {
    socket.on("connect", () => {
        setConnectionStatus("connected", true)
        restoreStateAfterReconnect()
    })
    socket.on("disconnect", reason => {
        setConnectionStatus(`disconnected (${reason})`, false)
    })
    socket.io.on("reconnect_attempt", attempt => {
        setConnectionStatus(`reconnecting (${attempt})`, false)
    })
    socket.io.on("reconnect", () => {
        setConnectionStatus("reconnected", true)
    })

    socket.on("ros2-topic-list", msg => quickLook.updateTopicList(socket, msg))
    socket.on("ros2-topic-field", msg => quickLook.updateTopicField(socket, msg))
    socket.on("ros2-message", msg => {
        graph().push(msg)
    })
    socket.on("ros2-subscribe", () => {
        pollObserverStatus()
    })
    socket.on("ros2-unsubscribe", () => {
        pollObserverStatus()
    })
    socket.on("ros2-status", msg => {
        const health = msg.health || {}
        const streams = msg.streams || []
        const pushInfo = (health.status_push_enabled)
            ? `, push=${health.status_push_mode || 'change+heartbeat'} (hb=${health.status_push_heartbeat_sec || 10.0}s)`
            : ""
        setObserverStatus(`clients=${health.active_clients || 0}, topics=${health.active_topics || 0}, streams=${health.active_streams || 0}${pushInfo}`)
        renderStreamStatuses(streams)
    })
}

function main() {
    setConnectionStatus("connecting", false)
    graph().setRole(currentRole, roleOptionsFromControls(currentRole))
    syncControlsFromRole(currentRole)

    $("#ros2-topic-list").click(() => requestTopicList(""))
    $("#total_power").click(() => requestTopicList("total_power"))
    $("#spectrum_decimated").click(() => requestTopicList("spectrum_decimated"))
    $("#2d-plot").click(() => requestTopicList("2d-plot"))
    $("#sis_iv").click(() => requestTopicList("sis_iv"))
    $(".apply-role-options").click(() => applyRoleOptions())

    bindSocketEvents()
    startStatusPolling()
    socket.connect()
}

$(document).ready(main)
