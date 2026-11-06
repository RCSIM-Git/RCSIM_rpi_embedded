"""PL: Udostępnia stronę mapy, lokalne REST API konfiguracji i WebSocket.

EN: Provides the map page, a local configuration REST API, and a WebSocket.

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
    """PL: Serwer aiohttp dla mapy, konfiguracji i strumienia telemetrii.

    Endpointy REST konfiguracji akceptują żądania wyłącznie z localhost.

    EN: An aiohttp server for the map, configuration, and telemetry stream.

    Configuration REST endpoints accept requests only from localhost.
    """

    def __init__(self, config_manager: ConfigManager, port: int = 8080):
        """PL: Konfiguruje aplikację aiohttp, trasy i rejestr WebSocketów.

        Args:
            config_manager: Manager używany do odczytu i zapisu konfiguracji.
            port: Port TCP, na którym serwer będzie nasłuchiwać.

        EN: Configures the aiohttp app, its routes, and WebSocket registry.

        Args:
            config_manager: Manager used to read and save configuration.
            port: TCP port on which the server listens.
        """
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
        """PL: Uruchamia serwer HTTP na skonfigurowanym porcie.

        EN: Starts the HTTP server on the configured port.
        """
        self.runner = web.AppRunner(self.app)
        await self.runner.setup()
        self.site = web.TCPSite(self.runner, "0.0.0.0", self.port)
        await self.site.start()
        self.logger.info(f"Web Service started on http://0.0.0.0:{self.port}")

    async def stop(self):
        """PL: Zamyka połączenia WebSocket i zatrzymuje runner serwera.

        EN: Closes WebSocket connections and stops the server runner.
        """
        for ws in self.websockets:
            await ws.close(code=1001, message="Server shutdown")
        if self.runner:
            await self.runner.cleanup()
        self.logger.info("Web Service stopped.")

    async def handle_index(self, request):
        """PL: Zwraca plik strony mapy albo odpowiedź błędu HTTP 500.

        Args:
            request: Żądanie HTTP aiohttp dla głównej trasy.

        EN: Returns the map page file or an HTTP 500 error response.

        Args:
            request: aiohttp HTTP request for the root route.
        """
        try:
            # Assumes map.html is in the project root or logic folder.
            # Adjust path as needed.
            path = os.path.join(os.path.dirname(os.path.dirname(__file__)), "map.html")
            return web.FileResponse(path)
        except Exception as e:
            return web.Response(text=f"Error loading map.html: {e}", status=500)

    async def handle_get_config(self, request):
        """PL: Zwraca bieżącą konfigurację wyłącznie klientom localhost.

        Args:
            request: Żądanie HTTP aiohttp z adresem klienta.

        EN: Returns the current configuration only to localhost clients.

        Args:
            request: aiohttp HTTP request containing the client address.
        """
        if request.remote not in ("127.0.0.1", "::1"):
            self.logger.warning(f"Blocked unauthorized GET config request from remote IP: {request.remote}")
            return web.json_response({"status": "ERROR", "message": "Access Denied. Localhost only."}, status=403)
        return web.json_response(self.config_manager.config)

    async def handle_post_config(self, request):
        """PL: Zapisuje konfigurację JSON z żądania klienta localhost.

        Niepoprawne dane zwracają odpowiedź HTTP 400, a błąd zapisu — HTTP 500.

        Args:
            request: Żądanie HTTP aiohttp zawierające konfigurację JSON.

        EN: Saves JSON configuration from a localhost client request.

        Invalid input returns HTTP 400, while a save failure returns HTTP 500.

        Args:
            request: aiohttp HTTP request containing the JSON configuration.
        """
        if request.remote not in ("127.0.0.1", "::1"):
            self.logger.warning(f"Blocked unauthorized POST config request from remote IP: {request.remote}")
            return web.json_response({"status": "ERROR", "message": "Access Denied. Localhost only."}, status=403)
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
        """PL: Rejestruje WebSocket do strumieniowania telemetrii.

        Wiadomości tekstowe od klienta są odbierane, ale nieprzetwarzane.

        Args:
            request: Żądanie HTTP aiohttp rozpoczynające połączenie WebSocket.

        EN: Registers a WebSocket connection for telemetry streaming.

        Text messages from the client are received but not processed.

        Args:
            request: aiohttp HTTP request that starts the WebSocket connection.
        """
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
        """PL: Serializuje dane telemetrii i wysyła je do połączonych klientów.

        Args:
            data: Słownik telemetrii do rozesłania.

        EN: Serializes telemetry data and sends it to connected clients.

        Args:
            data: Telemetry dictionary to broadcast.
        """
        if not self.websockets:
            return

        json_data = json.dumps(data)
        for ws in self.websockets:
            try:
                await ws.send_str(json_data)
            except Exception as e:
                self.logger.debug(f"WS send error: {e}")
