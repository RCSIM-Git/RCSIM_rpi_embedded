#!/usr/bin/env python3
"""PL: Tekstowy interfejs curses do podglądu telemetrii UDP.

EN: Curses-based text interface for viewing UDP telemetry.
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
    """PL: Inicjalizuje curses i zwraca główne okno terminala.

    EN: Initialize curses and return the main terminal window.

    Returns:
        Any: Główne okno curses / Main curses window.
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
    """PL: Obsługuje terminalowy podgląd telemetrii i informacji systemowych.

    EN: Provide a terminal view of telemetry and system information.
    """

    def __init__(self, stdscr: Any) -> None:
        """PL: Konfiguruje okno, gniazdo UDP i dane systemowe.

        EN: Configure the window, UDP socket, and system information.

        Args:
            stdscr: Główne okno curses / The main curses window.
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
        """PL: Odbiera pakiety UDP i aktualizuje dane wyświetlane w GUI.

        EN: Receive UDP packets and update the data displayed by the GUI.
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
        """PL: Rysuje nagłówek, informacje systemowe i telemetrię.

        EN: Draw the header, system information, and telemetry.
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
        """PL: Uruchamia wątek odbioru i odświeża interfejs terminala.

        EN: Start the receive thread and refresh the terminal interface.
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
    """PL: Tworzy GUI telemetrii i uruchamia jego pętlę.

    EN: Create the telemetry GUI and run its event loop.

    Args:
        stdscr: Główne okno curses / The main curses window.
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
