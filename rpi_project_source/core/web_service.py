"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.
"""
import json
import logging
import os
import weakref
from typing import Any

from aiohttp import WSMsgType, web
from core.config_loader import ConfigManager


class WebService:
    """
    Serwis WWW i WebSocket (aiohttp).
    Web and WebSocket Service (aiohttp).

    Funkcje / Features:
    - Serwuje / Serves `map.html`
    - REST API dla Config / REST API for Config
    - WebSocket streaming (Telemetry)
    """

    def __init__(self, config_manager: ConfigManager, port: int = 8080):
        self.config_manager = config_manager
        self.port = port
        self.logger = logging.getLogger("WebService")
        self.app = web.Application()
        self.runner = None
        self.site = None
        self.websockets = weakref.WeakSet()

        # Routes
        self.app.router.add_get("/", self.handle_index)
        self.app.router.add_get("/api/config", self.handle_get_config)
        self.app.router.add_post("/api/config", self.handle_post_config)
        self.app.router.add_get("/ws/telemetry", self.handle_telemetry_ws)

        # Serve tiles (optional, if we have local tiles)
        # self.app.router.add_static('/tiles', './tiles')

    async def start(self):
        """Starts the web server."""
        self.runner = web.AppRunner(self.app)
        await self.runner.setup()
        self.site = web.TCPSite(self.runner, "0.0.0.0", self.port)
        await self.site.start()
        self.logger.info(f"Web Service started on http://0.0.0.0:{self.port}")

    async def stop(self):
        """Stops the web server."""
        for ws in self.websockets:
            await ws.close(code=1001, message="Server shutdown")
        if self.runner:
            await self.runner.cleanup()
        self.logger.info("Web Service stopped.")

    async def handle_index(self, request):
        """Serves the main map page."""
        try:
            # Assumes map.html is in the project root or logic folder.
            # Adjust path as needed.
            path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "map.html")
            return web.FileResponse(path)
        except Exception as e:
            return web.Response(text=f"Error loading map.html: {e}", status=500)

    async def handle_get_config(self, request):
        """Returns current configuration."""
        return web.json_response(self.config_manager.config)

    async def handle_post_config(self, request):
        """Updates configuration."""
        try:
            new_config = await request.json()
            if self.config_manager.save_config(new_config):
                return web.json_response({"status": "OK", "message": "Config saved"})
            else:
                return web.json_response(
                    {"status": "ERROR", "message": "Failed to save"}, status=500
                )
        except Exception as e:
            return web.json_response({"status": "ERROR", "message": str(e)}, status=400)

    async def handle_telemetry_ws(self, request):
        """Handles WebSocket connection for telemetry streaming."""
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        self.websockets.add(ws)
        self.logger.info("New WebSocket connection")

        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    # Client messages (e.g. commands)?
                    pass
                elif msg.type == WSMsgType.ERROR:
                    self.logger.warning(
                        f"ws connection closed with exception {ws.exception()}"
                    )
        finally:
            self.logger.info("WebSocket connection closed")
            self.websockets.discard(ws)
        return ws

    async def broadcast_telemetry(self, data: dict[str, Any]):
        """Broadcasts telemetry data to all connected clients."""
        if not self.websockets:
            return

        json_data = json.dumps(data)
        for ws in self.websockets:
            try:
                await ws.send_str(json_data)
            except Exception as e:
                self.logger.debug(f"WS send error: {e}")
