"""PL: Ręczny scenariusz sprawdzający watchdog nadzorcy bezpieczeństwa.

EN: Manual scenario for checking the safety supervisor watchdog.
"""

import logging
import os
import sys
import time

# Add project root to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from core.safety_supervisor import SafetyState, SafetySupervisor


# Mock HardwareManager
class MockHardwareManager:
    """PL: Minimalny mock wyjść sprzętowych do próby watchdogu.

    EN: Minimal hardware-output mock for the watchdog scenario.
    """

    def __init__(self):
        """PL: Inicjalizuje mock z neutralnymi wartościami sterowania.

        EN: Initialize the mock with neutral control values.
        """
        self.throttle = 0.0
        self.steering = 0.0

    def set_throttle(self, val):
        """PL: Zapisuje i wypisuje zadaną wartość gazu.

        EN: Record and print the requested throttle value.
        """
        print(f"[MOCK HW] set_throttle({val})")
        self.throttle = val

    def set_steering(self, val):
        """PL: Zapisuje zadaną wartość skrętu.

        EN: Record the requested steering value.
        """
        self.steering = val


def test_watchdog():
    """PL: Sprawdza prawidłowy watchdog i reakcję po utracie odświeżania.

    EN: Verify normal watchdog operation and response after feeding stops.
    """
    logging.basicConfig(level=logging.INFO)

    mock_hw = MockHardwareManager()
    supervisor = SafetySupervisor()

    print("--- Starting Supervisor Watchdog ---")
    supervisor.start(mock_hw)

    # Simulating Normal Operation
    print(">>> Phase 1: Normal Operation (Feeding watchdog)")
    for i in range(5):
        supervisor.feed_watchdog()
        time.sleep(0.1)
        if supervisor.state == SafetyState.CRITICAL:
            print("❌ FAILURE: Watchdog triggered too early!")
            return

    print("✅ Phase 1 Passed.")

    # Simulating Freeze
    print(">>> Phase 2: Simulating Freeze (Stop feeding)")
    time.sleep(0.5)  # Wait longer than 0.25s timeout

    if supervisor.state == SafetyState.CRITICAL:
        print("✅ Phase 2 Passed: Watchdog triggered CRITICAL state.")
    else:
        print(f"❌ FAILURE: Watchdog did not trigger! State: {supervisor.state}")

    # Check if throttle was declared 0
    # Note: The thread runs asynchronously, strict verification of mock call might need a queue or shared var check
    # But standard output [MOCK HW] set_throttle(0.0) should be visible

    supervisor.running = False
    print("--- Test Finished ---")


if __name__ == "__main__":
    test_watchdog()
