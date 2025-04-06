import * as fs from "fs"
import * as path from "path"
import { NextApiRequest } from "next";


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
