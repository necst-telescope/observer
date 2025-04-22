'use client'

import { Dispatch, ReactNode, SetStateAction, useEffect, useState } from "react"
import { Line } from 'react-chartjs-2'
import {
    Chart as ChartJS,
    LineElement,
    PointElement,
    LinearScale,
    Title,
    CategoryScale,
    type ChartData,
    type ChartOptions,
} from 'chart.js'
import { faArrowsLeftRight } from '@fortawesome/free-solid-svg-icons'
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome'

import styles from './page.module.scss'
import { TopicSelector } from "@/components/TopicSelector"
import { useSocket } from "@/providers/SocketProvider"
import { useSnackbar } from "@/providers/SnackbarProvider"


ChartJS.register(LineElement, PointElement, LinearScale, Title, CategoryScale)


function AxisVariableSetter(props: {
    axis: string,
    value: string | null,
    setValue: Dispatch<SetStateAction<string | null>>,
}): ReactNode {
    return (
        <div
            onDragOver={e => e.preventDefault()}
            onDrop={e => props.setValue(e.dataTransfer.getData('text/plain'))}
            className={styles.topicInfo}
        >
            {props.axis}-axis: {props.value}
        </div>
    )
}

type DataType = [(string | number), (string | number)]

export default function QLookPage(): ReactNode {
    const { socket } = useSocket()
    const { notify } = useSnackbar()

    const [targetTopic, setTargetTopic] = useState<string | null>(null)
    const [referenceTopic, setReferenceTopic] = useState<string | null>(null)

    const [data, setData] = useState<DataType[]>([])

    useEffect(() => {
        socket?.on('message', (severity: Severity, message: string) => {
            console.log(`Received message: ${severity} - ${message}`)
            if (severity === 'success') { return }
            notify?.(severity, message)
        })
        socket?.on('ros2-message', function (topic: string, msg: any) {
            const [targetTopicName, targetField] = targetTopic?.split('::') || []
            const [referenceTopicName, referenceField] = referenceTopic?.split('::') || []
            const newData = [msg.time || data.length, msg[targetField]] as DataType  // TODO: Add support for 2d data
            console.log(newData, msg, msg.time)
            if (topic === targetTopicName) {
                setData(prevData => [...prevData, newData].slice(-3000))  // TODO: Add filter by timestamp
            }
        })

        return () => {
            socket?.off('message')
            socket?.off('ros2-message')
        }
    }, [socket, targetTopic, referenceTopic])

    // TODO: unsubscribe the old topic before subscribing to a new one
    useEffect(() => {
        socket?.emit('subscribe', targetTopic?.split('::')[0])
        return () => {
            socket?.emit('unsubscribe', targetTopic)
        }
    }, [targetTopic])

    useEffect(() => {
        socket?.emit('subscribe', referenceTopic?.split('::')[0])
        return () => {
            socket?.emit('unsubscribe', referenceTopic)
        }
    }, [referenceTopic])

    const chartData: ChartData<'line', DataType[], string> = {
        datasets: [{
            label: targetTopic || '',
            fill: false,
            borderColor: 'rgb(75, 192, 192)',
            data,
            tension: 0,
        }]
    }

    const duration = 30  // seconds
    const now = Date.now() / 1e3
    const chartOptions: ChartOptions<"line"> = {
        hover: { mode: "index", intersect: true },
        animation: { duration: 0 },
        scales: {
            x: {
                type: "linear",
                title: { display: true, text: referenceTopic || '(timestamp)' },
                min: now - duration,
                max: now,
                ticks: {
                    minRotation: 0,
                    maxRotation: 0,
                    callback: (value: any, idx: number, ticks: any[]) => {
                        if (referenceTopic) { return value }
                        const timeString = new Date(value * 1e3).toISOString()
                        return /[0-9-]T([0-9:]*)\.[0-9]{3}Z/.exec(timeString)![1] as string
                    }
                }
            },
            y: {
                title: { display: true, text: targetTopic || '(value)' },
            }
        }
    }

    function swapTopics() {
        setTargetTopic(referenceTopic)
        setReferenceTopic(targetTopic)
    }

    return (
        <div className={styles.container}>
            <TopicSelector />
            <div>
                <div className={styles.displayTopicInfo}>
                    <AxisVariableSetter axis='x' value={referenceTopic || '(timestamp)'} setValue={(topic) => { notify && notify('error', 'Setting variable for x-axis isn\'t supported yet'); setReferenceTopic(topic) }} />
                    <button
                        onClick={swapTopics}
                        disabled={!targetTopic}
                        title='Swap x and y axis'
                    >
                        <FontAwesomeIcon icon={faArrowsLeftRight} />
                    </button>
                    <AxisVariableSetter axis='y' value={targetTopic} setValue={setTargetTopic} />
                </div>
                <Line data={chartData} options={chartOptions} />
            </div>
        </div>
    )
}
