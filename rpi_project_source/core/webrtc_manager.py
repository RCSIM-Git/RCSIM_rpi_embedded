"""PL: Zarządza połączeniami WebRTC, kanałami danych i sygnalizacją HTTP.

EN: Manages WebRTC connections, data channels, and HTTP signaling.

Copyright (c) 2026 RCSIM / Mateusz Buzek
Licensed under the MIT License. See LICENSE file in the project root for full license information.
"""

import asyncio
import logging
import threading
from typing import Any, Callable

from aiohttp import web
from aiortc import RTCPeerConnection, RTCSessionDescription


class WebRTCManager:
    """PL: Obsługuje sygnalizację HTTP i wysyłanie danych przez WebRTC.

    Pętlę asyncio uruchamia w osobnym wątku.

    EN: Handles HTTP signaling and data transmission over WebRTC.

    Runs the asyncio loop in a separate thread.
    """

    def __init__(
        self, port: int = 8080, on_data_received: Callable[[str], None] | None = None
    ) -> None:
        """PL: Inicjalizuje serwer sygnalizacyjny i rejestry połączeń.

        Args:
            port: Port HTTP używany przez serwer sygnalizacyjny.
            on_data_received: Opcjonalny callback dla wiadomości przychodzących.

        EN: Initializes the signaling server and connection registries.

        Args:
            port: HTTP port used by the signaling server.
            on_data_received: Optional callback for incoming messages.
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
        """PL: Uruchamia wątek obsługujący pętlę asyncio i serwer HTTP.

        EN: Starts the thread that runs the asyncio loop and HTTP server.
        """
        self._thread = threading.Thread(target=self._run_async_loop, daemon=True)
        self._thread.start()
        self.logger.info(f"WebRTCManager started on port {self.port}")

    def stop(self) -> None:
        """PL: Zamyka połączenia WebRTC, zatrzymuje pętlę i dołącza wątek.

        EN: Closes WebRTC connections, stops the event loop, and joins the thread.
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
        """PL: Kolejkuje wysłanie tekstu przez otwarte kanały o wskazanej etykiecie.

        Args:
            data: Tekstowa wiadomość do wysłania.
            channel_label: Etykieta kanału docelowego.

        EN: Schedules the text for sending over open channels with the given label.

        Args:
            data: Text message to send.
            channel_label: Label of the target channel.
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
            """PL: Wysyła wiadomość, jeśli kanał nadal jest otwarty.

            EN: Sends the message if the channel is still open.
            """
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
        """PL: Tworzy pętlę asyncio, trasy sygnalizacyjne i opcjonalną stronę PWA.

        Następnie uruchamia serwer HTTP i obsługuje pętlę aż do zatrzymania.

        EN: Creates the asyncio loop, signaling routes, and optional PWA page.

        Then starts the HTTP server and runs the loop until shutdown.
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
                """PL: Serwuje główny plik aplikacji mobilnej.

                EN: Serves the main file of the mobile application.
                """
                return web.FileResponse(os.path.join(web_assets_path, "index.html"))

            app.router.add_get("/", handle_index)
            # Safe and built-in traversal mitigation using aiohttp's add_static
            app.router.add_static("/", path=web_assets_path, name="static")

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
        """PL: Zwraca nagłówki CORS dla żądania OPTIONS.

        Args:
            request: Żądanie HTTP aiohttp.

        Returns:
            Odpowiedź HTTP z nagłówkami CORS.

        EN: Returns CORS headers for an OPTIONS request.

        Args:
            request: aiohttp HTTP request.

        Returns:
            An HTTP response containing CORS headers.
        """
        origin = request.headers.get("Origin", "*")
        return web.Response(
            headers={
                "Access-Control-Allow-Origin": origin,
                "Access-Control-Allow-Methods": "POST, OPTIONS",
                "Access-Control-Allow-Headers": "Content-Type",
            }
        )

    async def _handle_offer(self, request: web.Request) -> web.Response:
        """PL: Negocjuje połączenie WebRTC na podstawie oferty SDP klienta.

        Args:
            request: Żądanie HTTP zawierające ofertę SDP.

        Returns:
            Odpowiedź JSON z SDP answer i nagłówkiem CORS.

        EN: Negotiates a WebRTC connection from the client's SDP offer.

        Args:
            request: HTTP request containing the SDP offer.

        Returns:
            A JSON response with the SDP answer and CORS header.
        """
        params = await request.json()
        self.logger.info(f"Incoming WebRTC Offer received from {request.remote}")
        offer = RTCSessionDescription(sdp=params["sdp"], type=params["type"])

        pc = RTCPeerConnection()
        self.logger.info("Created new RTCPeerConnection for incoming offer.")
        self._peer_connections.append(pc)  # Track all peer connections

        @pc.on("datachannel")
        def on_datachannel(channel: Any) -> None:
            """PL: Rejestruje kanał danych i podpina jego zdarzenia.

            Args:
                channel: Odebrany kanał danych WebRTC.

            EN: Registers the data channel and attaches its event handlers.

            Args:
                channel: Received WebRTC data channel.
            """
            lbl = channel.label
            self.logger.info(f"DataChannel received: {lbl}")
            if lbl not in self._data_channels:
                self._data_channels[lbl] = set()
            self._data_channels[lbl].add(channel)

            @channel.on("message")
            def on_message(message: Any) -> None:
                """PL: Przekazuje odebraną wiadomość do opcjonalnego callbacku.

                Args:
                    message: Odebrana wiadomość tekstowa lub binarna.

                EN: Passes the received message to the optional callback.

                Args:
                    message: Received text or binary message.
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
                """PL: Rejestruje kanał po otwarciu połączenia.

                EN: Registers the channel when the connection opens.
                """
                lbl = channel.label
                self.logger.info(f"DataChannel '{lbl}' OPEN")
                if lbl not in self._data_channels:
                    self._data_channels[lbl] = set()
                self._data_channels[lbl].add(channel)

            @channel.on("close")
            def on_close() -> None:
                """PL: Usuwa kanał z rejestru po zamknięciu.

                EN: Removes the channel from the registry when it closes.
                """
                lbl = channel.label
                self.logger.info(f"DataChannel '{lbl}' CLOSED")
                if lbl in self._data_channels:
                    self._data_channels[lbl].discard(channel)

        @pc.on("connectionstatechange")
        async def on_connectionstatechange() -> None:
            """PL: Obsługuje stan połączenia i usuwa zamknięte sesje.

            EN: Handles connection state changes and removes closed sessions.
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

        origin = request.headers.get("Origin", "*")
        return web.json_response(
            {"sdp": pc.localDescription.sdp, "type": pc.localDescription.type},
            headers={"Access-Control-Allow-Origin": origin},
        )

    async def _cleanup(self) -> None:
        """PL: Zamyka wszystkie połączenia i anuluje pozostałe zadania asyncio.

        EN: Closes all peer connections and cancels remaining asyncio tasks.
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
