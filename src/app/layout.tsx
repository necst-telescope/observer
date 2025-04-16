import { ReactNode } from 'react'
import './globals.scss'
import type { Metadata } from 'next'
import { SocketProvider } from "@/providers/SocketProvider"

export const metadata: Metadata = {
    title: 'Observer',
    description: 'NECST observation monitor',
}

export default function Layout(props: { children: ReactNode }): ReactNode {
    return (
        <html lang='en'>
            <body>
                <header>
                    <a href="/">Home</a>
                    <a href="/qlook">Q-Look</a>
                    <a href="/config">Configuration</a>
                </header>
                <SocketProvider>
                    {props.children}
                </SocketProvider>
            </body>
        </html>
    )
}
