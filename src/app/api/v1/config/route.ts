import * as fs from "fs"
import * as path from "path"
import { NextApiRequest } from "next";
import { Content, Puritan } from "next/font/google";
import { request } from "http";
import { error } from "console";


export async function GET(request: NextApiRequest): Promise<Response> {
    console.log(request.url)
    const configFileName = new URL(request.url as string).searchParams.get("filename")
    if (configFileName === null) {
        const configFilePath = path.join("/root", ".necst")
        const fileList = fs.readdirSync(configFilePath)

        return new Response(JSON.stringify(fileList), {
            headers: { 'content-type': 'text/plain' },
            status: 200,
        })
    }
    const configFilePath = path.join("/root/.necst", configFileName)
    const content = fs.readFileSync(configFilePath).toString()

    return new Response(content, {
        headers: { 'content-type': 'text/plain' },
        status: 200,
    })
}

export async function PUT(request: Request): Promise<Response> {
    try {
        console.log(request.url)
        const configFileName = new URL(request.url as string).searchParams.get("filename")
        const configFilePath = path.join("/root/.necst", configFileName as string)
        console.log(request.body)
        const content = await request.text()
        fs.writeFileSync(configFilePath, content)
        return new Response(undefined, {
            status: 200,
        })
    }
    catch (e) {
        return new Response(String(e), {
            status: 500,
        })
    }
}
