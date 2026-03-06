#!/usr/bin/env python3
"""LoRa Base Station web server.

Spawns the base_station_rpi binary, pipes its stdout to WebSocket clients,
and forwards commands from clients to its stdin.

Requires: pip install aiohttp
"""

import asyncio
import json
import os
import signal
import sys
from pathlib import Path

from aiohttp import web

HERE = Path(__file__).resolve().parent
BINARY = HERE.parent / "build" / "base_station_rpi"

clients: set[web.WebSocketResponse] = set()
process: asyncio.subprocess.Process | None = None
message_log: list[dict] = []
MAX_LOG = 200


async def broadcast(msg: dict):
    message_log.append(msg)
    if len(message_log) > MAX_LOG:
        del message_log[: len(message_log) - MAX_LOG]
    data = json.dumps(msg)
    dead = set()
    for ws in clients:
        try:
            await ws.send_str(data)
        except Exception:
            dead.add(ws)
    clients.difference_update(dead)


async def read_subprocess():
    global process
    if not BINARY.exists():
        print(f"ERROR: binary not found at {BINARY}", file=sys.stderr)
        print("Build it first: cd base_station_rpi/build && cmake .. && make", file=sys.stderr)
        return

    process = await asyncio.create_subprocess_exec(
        str(BINARY),
        stdin=asyncio.subprocess.PIPE,
        stdout=asyncio.subprocess.PIPE,
        stderr=asyncio.subprocess.PIPE,
    )

    async def read_stream(stream, is_stderr=False):
        while True:
            line = await stream.readline()
            if not line:
                break
            text = line.decode("utf-8", errors="replace").strip()
            if not text:
                continue
            try:
                obj = json.loads(text)
            except json.JSONDecodeError:
                obj = {"raw": text}
            if is_stderr:
                obj["_stderr"] = True
            obj["_dir"] = "rx"
            await broadcast(obj)

    await asyncio.gather(
        read_stream(process.stdout),
        read_stream(process.stderr, is_stderr=True),
    )

    code = await process.wait()
    await broadcast({"_dir": "rx", "error": f"base_station_rpi exited with code {code}"})
    process = None


async def send_to_process(cmd: str):
    if process and process.stdin:
        process.stdin.write((cmd + "\n").encode())
        await process.stdin.drain()


# --- HTTP handlers ---

async def handle_index(request):
    return web.FileResponse(HERE / "index.html")


async def handle_ws(request):
    ws = web.WebSocketResponse()
    await ws.prepare(request)
    clients.add(ws)

    # Send recent log to new client
    for msg in message_log[-50:]:
        await ws.send_str(json.dumps(msg))

    try:
        async for msg in ws:
            if msg.type == web.WSMsgType.TEXT:
                text = msg.data.strip()
                if not text:
                    continue
                try:
                    obj = json.loads(text)
                    obj["_dir"] = "tx"
                    await broadcast(obj)
                except json.JSONDecodeError:
                    pass
                await send_to_process(text)
    finally:
        clients.discard(ws)

    return ws


async def on_startup(app):
    app["radio_task"] = asyncio.create_task(read_subprocess())


async def on_cleanup(app):
    if process:
        process.terminate()
        try:
            await asyncio.wait_for(process.wait(), timeout=3)
        except asyncio.TimeoutError:
            process.kill()
    task = app.get("radio_task")
    if task:
        task.cancel()


def main():
    app = web.Application()
    app.on_startup.append(on_startup)
    app.on_cleanup.append(on_cleanup)
    app.router.add_get("/", handle_index)
    app.router.add_get("/ws", handle_ws)

    host = os.environ.get("HOST", "0.0.0.0")
    port = int(os.environ.get("PORT", "8080"))
    print(f"Starting server on http://{host}:{port}")
    web.run_app(app, host=host, port=port)


if __name__ == "__main__":
    main()
