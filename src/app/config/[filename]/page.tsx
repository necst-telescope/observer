"use client";

import { ReactNode, useState, useEffect, use } from "react";
import styles from "./page.module.scss"
import { useSnackbar } from "@/providers/SnackbarProvider"

export default function Page(props: {
    params: Promise<{ filename: string }>
}): ReactNode {
    const { filename } = use(props.params)
    const [content, setContent] = useState<string>("")
    const { notify } = useSnackbar()

    useEffect(() => {
        fetch(`/api/v1/config?filename=${filename}`)
            .then(res => res.text()) //ここのresはapi/v1/config/route.tsでリターンされたやつが入ってる
            .then(setContent)
    }, [filename])

    function handleChange(e: React.ChangeEvent<HTMLInputElement>) {
        setContent(e.target.value)
        console.log(e.target.value)
        fetch(`/api/v1/config?filename=${filename}`, {
            method: 'PUT',
            body: e.target.value
        }).then(res => res.status === 200 ? notify?.("success", "Saved!!") : notify?.("error", "Error!!"))
    }

    return (
        <div className={styles.main} >
            <input
                type="text"
                id="logo-text"
                value={content}
                onChange={handleChange}
            />
        </div>
    )

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
