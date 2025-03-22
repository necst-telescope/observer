import * as rclnodejs from 'rclnodejs'
import { Dispatch, SetStateAction, ReactNode, useEffect } from 'react'


class Msgs {
    topicName: string
    msgs: any[]

    constructor(topicName: string) {
        this.topicName = topicName
        this.msgs = []
    }

    push(msg: any) {
        this.msgs.push(msg)

        if (this.msgs.length > 100) {
            this.msgs.shift()
        }
    }

    get(): any[] {
        return this.msgs
    }
}


export class Client {
    node: rclnodejs.Node
    subscribers: { [topicName: string]: rclnodejs.Subscription }
    msgs: { [topicName: string]: Msgs }

    constructor(node: rclnodejs.Node) {
        this.subscribers = {}
        this.msgs = {}

        this.node = node
        node.spin()
    }

    static async new() {
        await rclnodejs.init()
        const node = new rclnodejs.Node('observer')
        return new Client(node)
    }

    subscribe(topic: string) {
        if (this.subscribers[topic] !== undefined) {
            return
        }

        const callback = (msg: any) => {
            this.msgs[topic].push(msg)
        }

        const messageType = this.#getMessageType(topic)
        const subscription = this.node.createSubscription(messageType, topic, (msg) => {
            callback(msg)
        })
        this.subscribers[topic] = subscription
        this.msgs[topic] = new Msgs(topic)
    }

    unsubscribe(topic: string) {
        this.node.destroySubscription(this.subscribers[topic])
        delete this.subscribers[topic]
        delete this.msgs[topic]
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
        for (const topic in this.subscribers) {
            this.unsubscribe(topic)
        }
        this.node.destroy()
    }
}

export function ROS2Connection(props: {
    children?: ReactNode,
    setClient: Dispatch<SetStateAction<Client | undefined>>,
}): ReactNode {
    useEffect(() => {
        Client.new().then(c => {
            props.setClient(c)
        })
        return rclnodejs.shutdown
    }, [])
    return props.children
}

export const ros2Client = await Client.new()
