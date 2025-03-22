"use client";

import { ReactNode, useState } from "react";

export default function Page(): ReactNode {
    const [config, setConfig] = useState("");

    async function getConfig(params: string) {
        const res = await fetch("api/vi/config");
        const filenames = await res.json();
        setConfig(filenames)
    }
    return (
        <div>
            <h1>Config</h1>
            <p>config: {config}</p>
            {filenames}
        </div>

    )
}
