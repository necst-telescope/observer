"use client";

import { ReactNode, useState, useEffect, use } from "react";
import styles from "./page.module.scss"

export default function Page(): ReactNode {
    const [filename, setFilename] = useState([])
    // const [content, setContent] = useState<string>("")

    useEffect(() => {
        fetch(`/api/v1/config`)
            .then(res => res.json())
            .then(setFilename)
    }, [])

    //     return <div>
    //         {content}
    //     </div>
    // }
    return (
        < div className={styles.main} >
            <h1>Config file</h1>
            <ul>{
                filename.map((fileName) => (
                    <li key={fileName}><a href={`/config/${fileName}`}>{fileName}</a></li>
                ))
            }
            </ul>
        </div >
    )
}
