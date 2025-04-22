"use client"

import {
    ReactNode,
    createContext,
    useContext,
    useEffect,
    useState,
} from "react"
import { type Socket, io } from "socket.io-client"

type SocketContextType = {
    socket: Socket | null
}

const SocketContext = createContext<SocketContextType>({
    socket: null,
})

export const useSocket = () => {
    return useContext(SocketContext)
}

export function SocketProvider({ children }: { children: ReactNode }) {
    const [socket, setSocket] = useState<Socket | null>(null)

    useEffect(() => {
        const connectSocket = async () => {
            const serverUrl = process.env.NEXT_PUBLIC_APP_URL!

            const socketInstance = io(serverUrl, {
                path: "/api/socket",
                autoConnect: true,
                reconnection: true,
            })

            socketInstance.on("connect", () => {
                console.debug("Connected to the server.")
            })

            socketInstance.on("disconnect", (reason) => {
                console.debug("Disconnected from the server.", reason)
            })

            setSocket(socketInstance)
        }

        connectSocket()

        return () => { socket?.disconnect() }
    }, [])

    return (
        <SocketContext.Provider value={{ socket }}>
            {children}
        </SocketContext.Provider>
    )
}
