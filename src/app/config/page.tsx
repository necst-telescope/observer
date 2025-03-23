"use client";

import { ReactNode, useState, useEffect } from "react";
import styles from "./page.module.scss"
import url from "../app/page.tsx"

export default function Page(): ReactNode {
    const [fileNames, setFileNames] = useState<string[]>([]);

    useEffect(() => {
        fetch("api/v1/config")
            .then(res => res.json())
            .then(data => setFileNames(data))
    }, [])

    return (
        <div className={styles.main}>
            <h1>Config file</h1>
            {fileNames.map((fileName) => (
                <ul>
                    <li><a key={fileName} href={url}>{fileName}</a></li>
                </ul>
            ))}
        </div >
    )
}
