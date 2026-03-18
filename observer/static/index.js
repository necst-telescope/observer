"use strict";

import * as quickLook from "./quick-look.js";
import { Graph } from "./chart.js";

function getTotalPowerRangeFromUI() {
  const startRaw = $("#tp-ch-start").val();
  const endRaw = $("#tp-ch-end").val();
  const start = startRaw === "" ? null : Number(startRaw);
  const end = endRaw === "" ? null : Number(endRaw);
  return {
    start: Number.isFinite(start) ? start : null,
    end: Number.isFinite(end) ? end : null,
  };
}

function applyTotalPowerRange(socket) {
  const graph = Graph("#chart", socket);
  const range = getTotalPowerRangeFromUI();
  graph.setTotalPowerChRange(range.start, range.end);
}

function main() {
  const socket = io("/qlook");
  const role = "";
  $("#ros2-topic-list").click(() => {
    Graph("#chart", socket).clear();
    socket.emit("ros2-topic-list-request", { role: role });
  });

  socket.on("ros2-topic-list", (msg) => quickLook.updateTopicList(socket, msg));
  socket.on("ros2-topic-field", (msg) =>
    quickLook.updateTopicField(socket, msg),
  );
  socket.on("ros2-message", (msg) => {
    Graph("#chart", socket).push(msg.topic_name, msg.data);
  });
}

function Sub() {
  const socket = io("/qlook");
  const role = "total_power";
  const applyRange = () => applyTotalPowerRange(socket);

  $("#total_power").click(() => {
    applyRange();
    socket.emit("ros2-topic-list-request", { role: role });
  });

  $("#tp-ch-apply").click(applyRange);
  $("#tp-ch-start").on("change", applyRange);
  $("#tp-ch-end").on("change", applyRange);

  socket.on("ros2-topic-list", (msg) => quickLook.updateTopicList(socket, msg));
  socket.on("ros2-topic-field", (msg) =>
    quickLook.updateTopicField(socket, msg),
  );
  socket.on("ros2-message", (msg) => {
    Graph("#chart", socket).push(msg.topic_name, msg.data, role);
  });
}

function sub2() {
  const socket = io("/qlook");
  const role = "2d-plot";
  $("#2d-plot").click(() => {
    // TODO: Update chart in 2D-plot mode from 1D-plot mode.
    Graph("#chart", socket).clear();
    socket.emit("ros2-topic-list-request", { role: role });
  });

  socket.on("ros2-topic-list", (msg) => quickLook.updateTopicList(socket, msg));
  // TODO: Do not display buttons of fields.
  socket.on("ros2-topic-field", (msg) =>
    quickLook.updateTopicField(socket, msg),
  );
  socket.on("ros2-message", (msg) => {
    Graph("#chart", socket).push(msg.topic_name, msg.data, role);
  });
}
function sub3() {
  const socket = io("/qlook");
  const role = "sis_iv";
  $("#sis_iv").click(() => {
    // TODO: Update chart in 2D-plot mode from 1D-plot mode.
    Graph("#chart", socket).clear();
    socket.emit("ros2-topic-list-request", { role: role });
  });

  socket.on("ros2-topic-list", (msg) => quickLook.updateTopicList(socket, msg));
  // TODO: Do not display buttons of fields.
  socket.on("ros2-topic-field", (msg) =>
    quickLook.updateTopicField(socket, msg),
  );
  socket.on("ros2-message", (msg) => {
    Graph("#chart", socket).push(msg.topic_name, msg.data, role);
  });
}

$(document).ready(main);
$(document).ready(Sub);
$(document).ready(sub2);
$(document).ready(sub3);
