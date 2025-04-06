"use client";

import { ReactNode, useState, useEffect, use } from "react";
import styles from "./page.module.scss"

export default function Page(props: {
    params: Promise<{ filename: string }>
}): ReactNode {
    const { filename } = use(props.params)
    const [content, setContent] = useState<string>("")

    useEffect(() => {
        fetch(`/api/v1/config?filename=${filename}`)
            .then(res => res.text())
            .then(setContent)
    }, [filename])

    return <div>
        {content}
    </div>
}
// return (
//     < div className={styles.main} >
//         <h1>Config file</h1>
//         {content}
//         {
//             filename.map((fileName) => (
//                 <ul>
//                     <li><a key={fileName} href="localhost:3000/config/{fileName}">{fileName}</a></li>
//                 </ul>
//             ))
//         }
//     </div >
// )
// }
