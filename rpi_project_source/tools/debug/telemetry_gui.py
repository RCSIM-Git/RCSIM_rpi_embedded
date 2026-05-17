#!/usr/bin/env python3
"""
Simple telemetry GUI for debugging purposes.
"""

import curses
import json
import logging
import select
import socket
import time
from threading import Thread
from typing import Any


# Initialize curses
def init_curses() -> Any:
    """
    Inicjalizuje interfejs biblioteki curses.
    Initializes the curses library interface.

    Returns:
        Any: Okno główne stdscr. / Main stdscr window.
    """
    stdscr = curses.initscr()
    curses.noecho()
    curses.cbreak()
    stdscr.keypad(True)
    stdscr.nodelay(True)
    curses.start_color()
    curses.init_pair(1, curses.COLOR_GREEN, curses.COLOR_BLACK)
    curses.init_pair(2, curses.COLOR_YELLOW, curses.COLOR_BLACK)
    curses.init_pair(3, curses.COLOR_RED, curses.COLOR_BLACK)
    return stdscr


# Late import to avoid issues with curses
from core.utils.system_info import get_board_info  # noqa: E402


class TelemetryGUI:
    """
    Klasa obsługująca interfejs tekstowy telemetrii (z użyciem curses).
    Class handling the text-based telemetry interface (using curses).
    """

    def __init__(self, stdscr: Any) -> None:
        """
        Inicjalizuje GUI telemetrii.
        Initializes the telemetry GUI.

        Args:
            stdscr (Any): Ekran tekstowy biblioteki curses. / Text screen of curses library.
        """
        self.stdscr = stdscr
        self.udp_ip = "0.0.0.0"
        self.udp_port = 54321
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind((self.udp_ip, self.udp_port))
        self.sock.setblocking(False)
        self.running = True
        self.data = {}
        self.board_info = get_board_info()

    def listen(self) -> None:
        """
        Nasłuchuje przychodzących pakietów danych po UDP.
        Listens for incoming data packets over UDP.
        """
        while self.running:
            try:
                # Use select to wait for data or timeout (0.1s)
                # This is more efficient than busy-waiting with sleep
                readable, _, _ = select.select([self.sock], [], [], 0.1)
                if readable:
                    data, addr = self.sock.recvfrom(1024)
                    self.data = json.loads(data.decode())
            except (socket.error, json.JSONDecodeError):
                pass

    def draw(self) -> None:
        """
        Rysuje elementy interfejsu (nagłówek, system info, dane).
        Draws interface elements (header, system info, data).
        """
        self.stdscr.clear()
        height, width = self.stdscr.getmaxyx()

        # Header
        title = "RCSIM Telemetry"
        self.stdscr.addstr(0, (width - len(title)) // 2, title, curses.A_BOLD)
        self.stdscr.addstr(1, 0, "=" * width)

        # System Info
        self.stdscr.addstr(3, 2, "System Info", curses.A_UNDERLINE)
        self.stdscr.addstr(4, 4, f"Board Model: {self.board_info['model_name']}")
        self.stdscr.addstr(5, 4, f"CPU Temp: {self.board_info['cpu_temp']:.1f}°C")
        self.stdscr.addstr(6, 4, f"RAM Usage: {self.board_info['ram_usage']:.1f}%")

        # Telemetry Data
        self.stdscr.addstr(8, 2, "Telemetry", curses.A_UNDERLINE)
        if self.data:
            row = 9
            for key, value in self.data.items():
                if isinstance(value, dict):
                    self.stdscr.addstr(row, 4, f"{key}:", curses.A_BOLD)
                    row += 1
                    for sub_key, sub_value in value.items():
                        self.stdscr.addstr(row, 6, f"{sub_key}: {sub_value}")
                        row += 1
                else:
                    self.stdscr.addstr(row, 4, f"{key}: {value}")
                    row += 1

        # Footer
        footer = "Press 'q' to quit"
        self.stdscr.addstr(height - 1, (width - len(footer)) // 2, footer)

        self.stdscr.refresh()

    def run(self) -> None:
        """
        Główna pętla uruchomieniowa programu.
        Main run loop of the program.
        """
        listen_thread = Thread(target=self.listen, daemon=True)
        listen_thread.start()

        while self.running:
            self.draw()
            try:
                if self.stdscr.getch() == ord("q"):
                    self.running = False
            except curses.error:
                pass
            time.sleep(0.1)


def main(stdscr: Any) -> None:
    """
    Główna funkcja wywoływana przez wrapper curses.
    Main function called by curses wrapper.

    Args:
        stdscr (Any): Ekran tekstowy biblioteki curses. / Text screen of curses library.
    """
    gui = TelemetryGUI(stdscr)
    gui.run()


if __name__ == "__main__":
    try:
        curses.wrapper(main)
    except Exception as e:
        logging.critical(f"Failed to run telemetry GUI: {e}")
        # Restore terminal state
        curses.nocbreak()
        curses.echo()
        curses.endwin()
