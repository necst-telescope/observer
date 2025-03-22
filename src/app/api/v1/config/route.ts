import * as fs from 'fs';

export async function GET(request: Request): Promise<Response> {
    console.log("GET", request.url);
    console.log(fs.readdirSync('/root/.necst/'));

    const filenames = fs.readdirSync('/root/.necst/');

    return new Response(JSON.stringify(filenames), {
        headers: { 'content-type': 'text/plain' },
        status: 200,
    })
}
