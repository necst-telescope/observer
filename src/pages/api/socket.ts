import type { Server as NetServer } from "node:http"
import type { Socket as SocketNet } from "node:net"

import type { NextApiRequest, NextApiResponse } from "next"
import { type Socket, Server as SocketIOServer } from "socket.io"

import { Client } from "@/lib/ros2"


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
    const ros2Client = Client.get()
    ros2Client.attach(io)

    io.on("connection", async (connection: Socket) => {
        console.debug(`Client ${connection.id} connected`)
        console.debug(io.engine.clientsCount)

        connection.on("get-topic-list", () => {
            console.debug(`Get get-topic-list from ${connection.id}`)
            try {
                const topics = ros2Client.listTopics()
                io.to(connection.id).emit("get-topic-list", ...topics)
                io.to(connection.id).emit("message", "success", 'Successfully got topic list')
            } catch (error) {
                console.error("Error getting topic list:", error)
                io.to(connection.id).emit("message", "error", `Failed to get topic list: ${error}`)
            }
        })

        connection.on("get-node-list", () => {
            console.debug(`Get get-node-list from ${connection.id}`)
            try {
                const nodes = ros2Client.listNodes()
                io.to(connection.id).emit("get-node-list", ...nodes)
                io.to(connection.id).emit("message", "success", 'Successfully got node list')
            } catch (error) {
                console.error("Error getting node list:", error)
                io.to(connection.id).emit("message", "error", `Failed to get node list: ${error}`)
            }
        })

        connection.on("get-message-fields", (topic: string) => {
            console.debug(`Get get-message-fields for ${topic} from ${connection.id}`)
            try {
                const fields = ros2Client.listMessageFields(topic)
                io.to(connection.id).emit("get-message-fields", topic, ...fields)
                io.to(connection.id).emit("message", "success", `Successfully got message fields for ${topic}`)
            } catch (error) {
                console.error(`Error getting message fields for ${topic}:`, error)
                io.to(connection.id).emit("message", "error", `Failed to get message fields for ${topic}: ${error}`)
            }
        })

        connection.on("get-topic-info", () => {
            console.debug(`Get get-topic-info from ${connection.id}`)
            try {
                const info: any = {}
                const topics = ros2Client.listTopics()
                topics.forEach((topic) => {
                    const messageFields = ros2Client.listMessageFields(topic)
                    info[topic] = messageFields
                })
                io.to(connection.id).emit("get-topic-info", info)
                io.to(connection.id).emit("message", "success", 'Successfully got topic info')
            } catch (error) {
                console.error(`Error getting topic info:`, error)
                io.to(connection.id).emit("message", "error", `Failed to get topic info: ${error}`)
            }
        })

        connection.on("subscribe", (topic: string) => {
            console.debug(`Get subscribe for ${topic} from ${connection.id}`)
            try {
                ros2Client.subscribe(topic)
                connection.join(topic)
                io.to(connection.id).emit("message", "success", `Successfully subscribed to topic ${topic}`)
            } catch (error) {
                console.error(`Error getting message for ${topic}:`, error)
                io.to(connection.id).emit("message", "error", `Failed to subscribe to ${topic}: ${error}`)
            }
        })

        connection.on("unsubscribe", (topic: string) => {
            console.debug(`Get unsubscribe for ${topic} from ${connection.id}`)
            connection.leave(topic)
            // ros2Client.unsubscribe(topic)  // TODO: Check all subscribers left
        })
    })

    res.socket.server.io = io

    res.end()
}
