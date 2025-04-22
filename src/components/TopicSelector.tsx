import { useEffect, useState, type ReactNode } from "react"
import { faRefresh } from '@fortawesome/free-solid-svg-icons'
import { FontAwesomeIcon } from '@fortawesome/react-fontawesome'
import classNames from 'classnames/bind'

import { useSocket } from "@/providers/SocketProvider"
import styles from './TopicSelector.module.scss'


enum NodeType {
    NAMESPACE,
    TOPIC,
    MESSAGE_FIELD,
}

type TopicTree = {
    [key: string]: [NodeType, TopicTree] | [NodeType, string]
}


function createTree(topicInfo: { [topic: string]: string[] }): TopicTree {
    const tree: any = {}

    for (const [topic, fields] of Object.entries(topicInfo)) {
        const parts = topic.split('/').filter(part => part !== '')

        // Remove last part of the topic name as it's the topic name,
        // not a part of namespace
        const lastPart = parts.pop()

        let current = tree

        for (let part of parts) {
            part = part.concat('/')
            if (!current[part]) {
                current[part] = [NodeType.NAMESPACE, {}]
            }
            current = current[part][1]
        }
        if (lastPart) {
            current[lastPart] = [NodeType.TOPIC, fields ? {} : topic]
        } else {
            continue
        }

        current = current[lastPart][1]
        if (fields) {
            for (const field of fields) {
                current[field] = [NodeType.MESSAGE_FIELD, lastPart.concat(`::${field}`)]
            }
        }
    }

    return tree
}


/**
 * Accordion component that can be opened and closed to show or hide its content.
 * @param props.title - The title of the block, which is always displayed
 * @param props.children - The content of the block, which is displayed only when the
 *                         accordion is open
 */
function NamespaceAccordion(props: { title: string, children: ReactNode }): ReactNode {
    const [selected, setSelected] = useState(false)
    const cx = classNames.bind(styles)

    function toggleSelected() {
        setSelected(!selected)
    }

    return (
        <div className={cx({ accordionContainer: true, open: selected, namespace: true })}>
            <div onClick={toggleSelected} className={styles.accordionTitle}>
                <span>{props.title}</span>
            </div>
            <div className={styles.accordion}>
                {props.children}
            </div>
        </div >
    )
}

/**
 * Accordion component that can be opened and closed to show or hide its content.
 * @param props.title - The title of the block, which is always displayed
 * @param props.children - The content of the block, which is displayed only when the
 *                         accordion is open
 */
function TopicAccordion(props: { title: string, children: ReactNode }): ReactNode {
    const [selected, setSelected] = useState(false)
    const cx = classNames.bind(styles)

    function toggleSelected() {
        setSelected(!selected)
    }

    return (
        <div className={cx({ accordionContainer: true, open: selected, topic: true })}>
            <div onClick={toggleSelected} className={styles.accordionTitle}>
                <span>{props.title}</span>
            </div>
            <div className={styles.accordion}>
                {props.children}
            </div>
        </div >
    )
}

/**
 * Accordion component that can be opened and closed to show or hide its content.
 * @param props.title - The title of the block, which is always displayed
 * @param props.children - The content of the block, which is displayed only when the
 *                         accordion is open
 * @param props.onClick - The function to call when the block is clicked
 */
function MessageFieldAccordion(props: {
    title: string, children?: ReactNode, qualname: string
}): ReactNode {
    const [selected, setSelected] = useState(false)
    const cx = classNames.bind(styles)

    // function toggleSelected() {
    //     setSelected(!selected)
    //     // TODO: if nested message type
    // }

    return (
        <div className={cx({ accordionContainer: true, open: selected, field: true })}>
            <div
                // onClick={toggleSelected}
                className={styles.accordionTitle}
                draggable
                onDragStart={e => e.dataTransfer.setData('text/plain', props.qualname)}
            >
                <span>{props.title}</span>
            </div>
            {/* <div className={styles.accordion}>  // TODO: correctly handle nested message types
                {props.children}
            </div> */}
        </div >
    )
}


function TreeAccordion(props: { tree: TopicTree, prefix?: string, level?: number }): ReactNode {
    const level = props.level || 0
    const prefix = props.prefix || '/'

    return (
        <div className={styles.tree}>
            {Object.entries(props.tree).map(([key, [type, value]]) => (
                type === NodeType.MESSAGE_FIELD ? (
                    <MessageFieldAccordion key={key} title={key} qualname={`${prefix}::${key}`} />
                ) : typeof value === 'string' ? (
                    <div key={key}>{value}</div>
                ) : type === NodeType.NAMESPACE ? (
                    <NamespaceAccordion key={key} title={key}>
                        <TreeAccordion tree={value} level={level + 1} prefix={`${prefix}${key}`} />
                    </NamespaceAccordion>
                ) : type === NodeType.TOPIC ? (
                    <TopicAccordion key={key} title={key}>
                        <TreeAccordion tree={value} level={level + 1} prefix={`${prefix}${key}`} />
                    </TopicAccordion>
                )
                    : null
            ))}
        </div>
    )
}


export function TopicSelector(props: {}): ReactNode {
    const { socket } = useSocket()

    const [messagesInfo, setMessagesInfo] = useState<{ [topic: string]: string[] }>({})

    useEffect(() => {
        socket?.on("get-topic-info", (info) => {
            setMessagesInfo(info)
            console.debug("Response for get-topic-info:", info)
        })

        socket?.emit("get-topic-info")
        const autoUpdateTopicList = setInterval(() => {
            socket?.emit("get-topic-info")
        }, 10000)

        return () => {
            socket?.off("get-message-fields")
            socket?.off("get-topic-info")
            clearInterval(autoUpdateTopicList)
        }
    }, [socket])

    return (
        <div className={styles.parameterSelector}>
            <TreeAccordion tree={createTree(messagesInfo)} />
        </div>
    )
}
