import { ReactNode } from 'react'
import './globals.scss'
import type { Metadata } from 'next'
import { SocketProvider } from "@/providers/SocketProvider"
import { SnackbarProvider } from "@/providers/SnackbarProvider"
import { Quicksand } from 'next/font/google'

const font = Quicksand({ subsets: ['latin'] })

export const metadata: Metadata = {
    title: 'Observer',
    description: 'NECST observation monitor',
}

export default function Layout(props: { children: ReactNode }): ReactNode {
    return (
        <html lang='en'>
            <body className={font.className}>
                <header>
                    <a href="/">Home</a>
                    <a href="/qlook">Q-Look</a>
                    <a href="/config">Configuration</a>
                </header>
                <SnackbarProvider>
                    <SocketProvider>
                        {props.children}
                    </SocketProvider>
                </SnackbarProvider>
            </body>
        </html>
    )
}
