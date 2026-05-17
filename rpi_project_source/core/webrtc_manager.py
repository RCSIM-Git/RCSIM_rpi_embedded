"""
Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.

Menedżer WebRTC (WebRTC Manager).

Zarządza komunikacją WebRTC DataChannel i sygnalizacją HTTP.
Manages WebRTC DataChannel communication and HTTP Signaling.
"""

import asyncio
import logging
import threading
from typing import Any, Callable

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription


class WebRTCManager:
    """
    Zarządza komunikacją WebRTC DataChannel i sygnalizacją HTTP.
    Manages WebRTC DataChannel communication and HTTP Signaling.

    Uruchamia pętlę asyncio w wątku tła, aby nie blokować głównej pętli.
    Runs asyncio loop in background thread to avoid blocking main sync loop.
    """

    def __init__(
        self, port: int = 8080, on_data_received: Callable[[str], None] | None = None
    ) -> None:
        """
        Inicjalizuje menedżera WebRTC.
        Initializes the WebRTC manager.

        Args:
            port (int): Port dla serwera Http / HTTP port.
            on_data_received: Callback dla dancyh / data callback.
        """
        self.port = port
        self.on_data_received = on_data_received
        self.logger = logging.getLogger("WebRTCManager")
        self._loop: asyncio.AbstractEventLoop | None = None
        self._thread: threading.Thread | None = None
        self._shutdown_event = threading.Event()
        self._peer_connections: list[RTCPeerConnection] = []
        # Przechowujemy kanały w słowniku: dict[label, set[RTCDataChannel]]
        self._data_channels: dict[str, set] = {
            "control": set(),
            "telemetry": set(),
            "imu": set(),
            "gps": set(),
            "lidar": set(),
            "slam": set(),
            "battery": set(),
        }

    def start(self) -> None:
        """
        Uruchamia usługę WebRTC/HTTP w wątku tła.
        Starts the WebRTC/HTTP service in a background thread.
        """
        self._thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self._thread.start()
        self.logger.info(f"WebRTCManager started on port {self.port}")

    def stop(self) -> None:
        """
        Zatrzymuje usługę.
        Stops the service.
        """
        self._shutdown_event.set()
        if self._loop and self._loop.is_running():
            # Schedule cleanup and wait for it to complete
            future = asyncio.run_coroutine_threadsafe(self._cleanup(), self._loop)
            try:
                future.result(timeout=5.0)  # Wait up to 5 seconds for cleanup
            except Exception as e:
                self.logger.error(f"Error during cleanup: {e}")

            # Schedule loop stop after cleanup is done
            if self._loop and self._loop.is_running():
                self._loop.call_soon_threadsafe(self._loop.stop)

        if self._thread:
            self._thread.join(timeout=2.0)

    def send_data(self, data: str, channel_label: str = "telemetry") -> None:
        """
        Wysyła dane przez kanał danych o podanej etykiecie.
        Sends data through data channels with the specified label.

        Args:
            data (str): Dane tekstowe do wysłania. / Text data to send.
            channel_label (str): Nazwa kanału docelowego (np. 'telemetry', 'lidar').
        """
        if not self._loop or not data:
            return

        target_channels = self._data_channels.get(channel_label, set())
        if not target_channels:
            # Fallback - if specific channel not found/open, maybe we have at least one open somewhere?
            # Or just silently drop to prevent blocking.
            if len(data) < 150:  # Log only small packets to unspam lidar
                self.logger.debug(
                    f"DEBUG: Drop send_data for '{channel_label}', no channels open. Available: {list(self._data_channels.keys())}"
                )
            return

        def _safe_send(chan, msg):
            if chan.readyState == "open":
                try:
                    chan.send(msg)
                except Exception:
                    # Silently drop if channel closed between check and send
                    pass

        # Pętla po wszystkich otwartych kanałach o danej etykiecie
        for channel in list(target_channels):
            self._loop.call_soon_threadsafe(_safe_send, channel, data)

    def _run_async_loop(self) -> None:
        """
        Wewnętrzna metoda uruchamiająca pętlę asyncio.
        Internal method to run the asyncio loop.
        """
        self._loop = asyncio.new_event_loop()
        asyncio.set_event_loop(self._loop)

        import os

        app = web.Application()
        app.router.add_post("/offer", self._handle_offer)
        app.router.add_options("/offer", self._handle_options)

        # Serve PWA Mobile Web App
        base_dir = os.path.dirname(__file__)
        web_assets_path = os.path.abspath(
            os.path.join(base_dir, "..", "web_assets", "mobile")
        )

        if os.path.exists(web_assets_path):

            async def handle_index(request):
                return web.FileResponse(os.path.join(web_assets_path, "index.html"))

            async def handle_static(request):
                filename = request.match_info.get("filename", "")
                filepath = os.path.join(web_assets_path, filename)
                # Security check to prevent directory traversal
                abs_filepath = os.path.abspath(filepath)
                if abs_filepath.startswith(web_assets_path) and os.path.isfile(
                    filepath
                ):
                    return web.FileResponse(filepath)
                return web.Response(status=404)

            app.router.add_get("/", handle_index)
            app.router.add_get("/{filename}", handle_static)

        runner = web.AppRunner(app)
        self._loop.run_until_complete(runner.setup())
        site = web.TCPSite(runner, "0.0.0.0", self.port)

        self.logger.info(f"Starting HTTP Server on 0.0.0.0:{self.port}")
        self._loop.run_until_complete(site.start())

        # Keep running until stop requested
        try:
            self._loop.run_forever()
        except Exception as e:
            self.logger.error(f"Async loop exited with error: {e}")
        finally:
            self._loop.run_until_complete(runner.cleanup())
            self._loop.close()

    async def _handle_options(self, request: web.Request) -> web.Response:
        """
        Obsługuje żądania OPTIONS dla CORS.
        Handles OPTIONS requests for CORS.

        Args:
            request (web.Request): Żądanie HTTP. / HTTP request.

        Returns:
            web.Response: Odpowiedź HTTP. / HTTP response.
        """
        return web.Response(
            headers={
                "Access-Control-Allow-Origin": "*",
                "Access-Control-Allow-Methods": "POST, OPTIONS",
                "Access-Control-Allow-Headers": "Content-Type",
            }
        )

    async def _handle_offer(self, request: web.Request) -> web.Response:
        """
        Obsługuje ofertę WebRTC (SDP Offer) i zwraca odpowiedź (SDP Answer).
        Handles WebRTC offer (SDP Offer) and returns answer (SDP Answer).

        Args:
            request (web.Request): Żądanie HTTP z ofertą. / HTTP request with offer.

        Returns:
            web.Response: Odpowiedź HTTP z ofertą SDP. / HTTP response with SDP answer.
        """
        params = await request.json()
        self.logger.info(f"Incoming WebRTC Offer received from {request.remote}")
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        pc = RTCPeerConnection()
        self.logger.info("Created new RTCPeerConnection for incoming offer.")
        self._peer_connections.append(pc)  # Track all peer connections

        @pc.on("datachannel")
        def on_datachannel(channel: Any) -> None:
            """
            Callback wywoływany przy odebraniu DataChannel.
            Callback executed when receiving a DataChannel.

            Args:
                channel (Any): Kanał danych. / Data channel.
            """
            lbl = channel.label
            self.logger.info(f"DataChannel received: {lbl}")
            if lbl not in self._data_channels:
                self._data_channels[lbl] = set()
            self._data_channels[lbl].add(channel)

            @channel.on("message")
            def on_message(message: Any) -> None:
                """
                Callback wiadomości przychodzącej.
                Incoming message callback.

                Args:
                    message (Any): Odebrana wiadomość. / Received message.
                """
                # Call the external callback
                if self.on_data_received:
                    try:
                        # Ensure we don't block the async loop, but callback is likely fast or should be offloaded
                        self.on_data_received(message)
                    except Exception as e:
                        self.logger.error(f"Error in data callback: {e}")

            @channel.on("open")
            def on_open() -> None:
                lbl = channel.label
                self.logger.info(f"DataChannel '{lbl}' OPEN")
                if lbl not in self._data_channels:
                    self._data_channels[lbl] = set()
                self._data_channels[lbl].add(channel)

            @channel.on("close")
            def on_close() -> None:
                lbl = channel.label
                self.logger.info(f"DataChannel '{lbl}' CLOSED")
                if lbl in self._data_channels:
                    self._data_channels[lbl].discard(channel)

        @pc.on("connectionstatechange")
        async def on_connectionstatechange() -> None:
            """
            Callback zmiany stanu połączenia.
            Connection state change callback.
            """
            self.logger.info(f"Connection state change: {pc.connectionState}")
            if pc.connectionState == "failed":
                await pc.close()
            if pc.connectionState in ["failed", "closed"]:
                if pc in self._peer_connections:
                    self._peer_connections.remove(pc)

        # Handle the offer
        await pc.setRemoteDescription(offer)
        answer = await pc.createAnswer()
        await pc.setLocalDescription(answer)

        return web.json_response(
            {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type},
            headers={"Access-Control-Allow-Origin": "*"},
        )

    async def _cleanup(self) -> None:
        """
        Czyści zasoby asynchroniczne.
        Cleans up asynchronous resources.
        """
        self.logger.info("Starting WebRTC cleanup...")

        # 1. Close all peer connections
        for pc in self._peer_connections:
            try:
                await pc.close()
            except Exception as e:
                self.logger.warning(f"Error closing peer connection: {e}")
        self._peer_connections.clear()

        # 2. Cancel all pending tasks (except current cleanup task)
        current_task = asyncio.current_task()
        pending_tasks = [
            task
            for task in asyncio.all_tasks(self._loop)
            if task is not current_task and not task.done()
        ]

        if pending_tasks:
            self.logger.info(f"Cancelling {len(pending_tasks)} pending tasks...")
            for task in pending_tasks:
                task.cancel()

            # Wait for tasks to complete cancellation (with timeout)
            try:
                await asyncio.wait(pending_tasks, timeout=2.0)
            except Exception as e:
                self.logger.warning(f"Error waiting for task cancellation: {e}")

        # 3. Stop the event loop
        # 3. Stop the event loop is now handled in stop()
        # self._loop.stop()
        self.logger.info("WebRTC cleanup completed.")
