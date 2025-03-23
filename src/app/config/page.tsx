"use client";

import { ReactNode, useState, useEffect } from "react";
import styles from "./page.module.scss"

export default function Page(): ReactNode {
    const [fileNames, setFileNames] = useState<string[]>([]);

    useEffect(() => {
        fetch("api/v1/config")
            .then(res => res.json())
            .then(data => setFileNames(data))
    }, [])

    const files = fileNames.map((fileName) => ({
        slug: fileName.slug,
    }))

    return (
        <div className={styles.main}>
            <h1>Config file</h1>
            {fileNames.map((fileName) => (
                <ul>
                    <li><a key={fileName} href={"/config/${fileName}"}>{fileName}</a></li>
                </ul>
            ))}
        </div >
    )
}
