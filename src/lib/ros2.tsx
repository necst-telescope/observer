import * as rclnodejs from 'rclnodejs'
import type { Server as SocketIOServer } from "socket.io"


export class Client {
    private static instance: Client

    node: rclnodejs.Node
    subscribers: Map<string, rclnodejs.Subscription>
    // msgs: Map<string, Msgs>
    socket: SocketIOServer | null = null

    private constructor(node: rclnodejs.Node) {
        this.subscribers = new Map()

        this.node = node
        node.spin()
    }

    static async new() {
        if (!Client.instance) {
            await rclnodejs.init()
            const node = new rclnodejs.Node('observer')
            const instance = new Client(node)
            Client.instance = instance
        }
        return Client.instance
    }

    static get() {
        const timeout = 10  // seconds
        const start = Date.now()
        while (!Client.instance && (Date.now() - start) < timeout * 1000) {
            setTimeout(() => { }, 10)
        }

        if (!Client.instance) {
            throw new Error(`Failed to initialize ROS2 client in ${timeout} seconds`)
        }

        return Client.instance
    }

    attach(socket: SocketIOServer) {
        const instance = Client.get()
        instance.socket = socket
    }

    subscribe(topic: string) {
        if (this.subscribers.get(topic) !== undefined) {
            return
        }

        const callback = (msg: any) => {
            this.socket?.to(topic).emit("ros2-message", topic, msg)
        }

        const messageType = this.#getMessageType(topic)
        const options = {
            qos: new rclnodejs.QoS(
                rclnodejs.QoS.HistoryPolicy.RMW_QOS_POLICY_HISTORY_KEEP_LAST,
                1,
                rclnodejs.QoS.ReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_BEST_EFFORT,
                rclnodejs.QoS.DurabilityPolicy.RMW_QOS_POLICY_DURABILITY_VOLATILE,
            )
        }
        const subscription = this.node.createSubscription(messageType, topic, options, callback)
        this.subscribers.set(topic, subscription)
    }

    unsubscribe(topic: string) {
        if (!this.subscribers.has(topic)) {
            return
        }
        this.node.destroySubscription(this.subscribers.get(topic)!)
        this.subscribers.delete(topic)
    }

    #getMessageType(topic: string): keyof rclnodejs.MessagesMap {
        const allTopics = this.node.getTopicNamesAndTypes()
        const thisTopicMetadata = allTopics.find(t => t.name === topic)
        if (!thisTopicMetadata) {
            throw new Error(`Topic ${topic} not found`)
        }

        const messageType = thisTopicMetadata.types[0] as keyof rclnodejs.MessagesMap
        return messageType
    }

    destroy() {
        console.info('Destroying ROS2 client...')

        for (const topic in this.subscribers) {
            this.unsubscribe(topic)
        }
        this.node.destroy()
    }

    listTopics(): string[] {
        const allTopics = this.node.getTopicNamesAndTypes()
        return allTopics.map(t => t.name)
    }

    listNodes(): string[] {
        const allNodes = this.node.getNodeNames()
        return allNodes
    }

    listMessageFields(topic: string): string[] {
        const messageType = this.#getMessageType(topic)
        const msgObj = rclnodejs.createMessageObject(messageType)
        // console.table(Object.keys(msgObj).map((k: string) => [k, typeof msgObj[k] === 'object' ? Object.keys(msgObj[k]) : typeof msgObj[k]]))
        // console.table(new Map(Object.entries(msgObj).map(([k, v]) => [k, typeof v === 'object' ? Object.keys(v) : typeof v])))
        return Object.keys(msgObj)
    }
}


await Client.new()
