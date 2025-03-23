import * as fs from "fs"
import * as path from "path"
import { NextApiRequest } from "next";


export async function GET(request: NextApiRequest): Response {
    const configFileName = request.query.filename as string;
    const configFilePath = path.join("/root/.necst/config", configFileName);
    const content = fs.readFileSync(configFilePath).toString()

    return new Response(content, {
        headers: { 'content-type': 'text/plain' },
        status: 200,
    })
}
