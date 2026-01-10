'use client'

import { useEffect, useState } from "react"
import { Line } from 'react-chartjs-2'
import {
    Chart as ChartJS,
    LineElement,
    PointElement,
    LinearScale,
    CategoryScale,
    Tooltip,
    Legend,
    type ChartData,
    type ChartOptions,
} from 'chart.js'

import styles from './page.module.scss'
import { useSocket } from "@/providers/SocketProvider"
import { useSnackbar } from "@/providers/SnackbarProvider"

ChartJS.register(
    LineElement,
    PointElement,
    LinearScale,
    CategoryScale,
    Tooltip,
    Legend
)

type XYPoint = { x: number, y: number }
type TempPoint = [number, number]

// -------------------------
// ダミーデータ
// -------------------------
const DUMMY_AZ = 145.2
const DUMMY_EL = 32.8

const now = Date.now() / 1000

const DUMMY_POINTS: XYPoint[] = Array.from({ length: 10 }).map((_, i) => ({
    x: 140 + i * 0.5,
    y: 30 + i * 0.3,
}))

const DUMMY_TEMP: TempPoint[] = Array.from({ length: 10 }).map((_, i) => [
    now - (10 - i),
    12.0 + i * 0.05,
])

export default function HomePage() {
    const { socket } = useSocket()
    const { notify } = useSnackbar()

    const [az, setAz] = useState<number | null>(DUMMY_AZ)
    const [el, setEl] = useState<number | null>(DUMMY_EL)
    const [points, setPoints] = useState<XYPoint[]>(DUMMY_POINTS)
    const [tempData, setTempData] = useState<TempPoint[]>(DUMMY_TEMP)

    useEffect(() => {
        if (!socket) return

        socket.on('message', (severity: Severity, message: string) => {
            if (severity !== 'success') notify?.(severity, message)
        })

        socket.on('ros2-message', (topic: string, msg: any) => {
            const time = msg.time || Date.now() / 1000

            if (topic === "telescope/azimuth") setAz(msg.value)
            if (topic === "telescope/elevation") setEl(msg.value)

            if (topic === "telescope/temperature") {
                const value = msg.temp
                setTempData(prev => {
                    const oldArray: TempPoint[] = prev ?? []
                    const newPoint: TempPoint = [time, value]
                    return [...oldArray, newPoint].slice(-3000)
                })
            }

            if (az !== null && el !== null) {
                setPoints(prev => [...prev, { x: az, y: el }].slice(-3000))
            }
        })

        socket.emit('subscribe', "telescope/azimuth")
        socket.emit('subscribe', "telescope/elevation")
        socket.emit('subscribe', "telescope/temperature")

        return () => {
            socket.off('message')
            socket.off('ros2-message')
            socket.emit('unsubscribe', "telescope/azimuth")
            socket.emit('unsubscribe', "telescope/elevation")
            socket.emit('unsubscribe', "telescope/temperature")
        }
    }, [socket, az, el])

    // 表示用の値
    const displayAz = (az ?? DUMMY_AZ).toFixed(2)
    const displayEl = (el ?? DUMMY_EL).toFixed(2)

    let latestTemp = DUMMY_TEMP[DUMMY_TEMP.length - 1][1]
    if (tempData.length > 0) latestTemp = tempData[tempData.length - 1][1]
    const displayTemp = latestTemp.toFixed(2)

    // グラフデータ
    const azelData: ChartData<'line', XYPoint[], unknown> = {
        datasets: [
            {
                label: "Azimuth vs Elevation",
                data: points,
                borderColor: 'rgb(75, 192, 192)',
                backgroundColor: 'rgb(75, 192, 192)',
                showLine: true,
                tension: 0,
            }
        ]
    }

    const azelOptions: ChartOptions<'line'> = {
        plugins: { legend: { labels: { color: "#fff" } } },
        scales: {
            x: {
                type: 'linear',
                title: { display: true, text: 'Azimuth', color: "#fff" },
                ticks: { color: "#fff" },
                grid: { color: "rgba(255,255,255,0.2)" }
            },
            y: {
                type: 'linear',
                title: { display: true, text: 'Elevation', color: "#fff" },
                ticks: { color: "#fff" },
                grid: { color: "rgba(255,255,255,0.2)" }
            }
        }
    }

    const tempChartData: ChartData<'line', TempPoint[], string> = {
        datasets: [
            {
                label: "Temperature",
                data: tempData,
                borderColor: 'rgb(255, 99, 132)',
                fill: false,
                tension: 0,
            }
        ]
    }

    const tempChartOptions: ChartOptions<'line'> = {
        plugins: { legend: { labels: { color: "#fff" } } },
        scales: {
            x: {
                type: "linear",
                min: now - 30,
                max: now,
                title: { display: true, text: "Timestamp", color: "#fff" },
                ticks: {
                    color: "#fff",
                    callback: (value: any) => {
                        const t = new Date(value * 1000).toISOString()
                        return t.split("T")[1].split(".")[0]
                    }
                },
                grid: { color: "rgba(255,255,255,0.2)" }
            },
            y: {
                title: { display: true, text: "Temperature", color: "#fff" },
                ticks: { color: "#fff" },
                grid: { color: "rgba(255,255,255,0.2)" }
            }
        }
    }

    return (
        <div className={styles.container}>
            <h1 className={styles.title}>HOME Dashboard</h1>

            <div className={styles.values}>
                <div className={styles.valueItem}>
                    <span className={styles.label}>Azimuth</span>
                    <span className={styles.number}>{displayAz} deg</span>
                </div>

                <div className={styles.valueItem}>
                    <span className={styles.label}>Elevation</span>
                    <span className={styles.number}>{displayEl} deg</span>
                </div>

                <div className={styles.valueItem}>
                    <span className={styles.label}>Temperature</span>
                    <span className={styles.number}>{displayTemp} °C</span>
                </div>
            </div>

            <div className={styles.grid}>
                <div className={`${styles.chartCard} ${styles.azel}`}>
                    <h3>Azimuth vs Elevation (Trajectory)</h3>
                    <Line data={azelData} options={azelOptions} />
                </div>

                <div className={`${styles.chartCard} ${styles.temp}`}>
                    <h3>Temperature (Time Series)</h3>
                    <Line data={tempChartData} options={tempChartOptions} />
                </div>
            </div>
        </div>
    )
}
