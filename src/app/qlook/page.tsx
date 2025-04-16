'use client'

import { ReactNode, useEffect, useState } from "react"
import { faRefresh } from '@fortawesome/free-solid-svg-icons'
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome'

import { useSocket } from "@/providers/SocketProvider"


export default function QLookPage(): ReactNode {
    const { socket } = useSocket();

    const [topics, setTopics] = useState<string[]>([]);

    useEffect(() => {
        socket?.on("get-topic-list", (...topics: string[]) => {
            setTopics(topics)
            console.debug("Response for get-topic-list:", topics)
        })

        socket?.emit("get-topic-list")

        return () => { socket?.off("get-topic-list") }
    }, [socket])

    function refreshTopics() {
        socket?.emit("get-topic-list")
    }

    return (
        <div>
            {topics.map((topic, index) => (
                <button key={index} className="topic-button">
                    {topic}
                </button>
            ))}
            <button onClick={refreshTopics} disabled={!socket?.connected}>
                <FontAwesomeIcon icon={faRefresh} />
            </button>
        </div>
    )
}
