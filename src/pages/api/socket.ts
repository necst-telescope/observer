import type { Server as NetServer } from "node:http"
import type { Socket as SocketNet } from "node:net"

import type { NextApiRequest, NextApiResponse } from "next"
import { type Socket, Server as SocketIOServer } from "socket.io"

import { ros2Client } from "@/lib/ros2"


type NextApiResponseWithSocket = NextApiResponse & {
    socket: SocketNet & {
        server: NetServer & {
            io?: SocketIOServer
        }
    }
}


export default async function ioHandler  (
    req: NextApiRequest,
    res: NextApiResponseWithSocket,
)  {
    if (res.socket.server.io) {
        res.send("socket server is already running")
        return
    }

    const httpServer: NetServer = res.socket.server
    const io = new SocketIOServer(httpServer, { path: "/api/socket" })

    io.on("connection", async (connection: Socket) => {
        console.debug(`Client ${connection.id} connected`)
        console.log(io.engine.clientsCount)

        connection.on("get-topic-list", () => {
            console.debug(`Get get-topic-list from ${connection.id}`)
            const topics = ros2Client.listTopics()
            io.to(connection.id).emit("get-topic-list", ...topics)
        })

        connection.on("get-node-list", () => {
            console.debug(`Get get-node-list from ${connection.id}`)
            const nodes = ros2Client.listNodes()
            io.to(connection.id).emit("get-node-list", ...nodes)
        })
    })

    res.socket.server.io = io

    res.end()
}
