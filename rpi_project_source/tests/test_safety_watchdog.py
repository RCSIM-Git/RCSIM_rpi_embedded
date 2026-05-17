import logging
import os
import sys
import time

# Add project root to sys.path
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), "..")))

from core.safety_supervisor import SafetyState, SafetySupervisor


# Mock HardwareManager
class MockHardwareManager:
    def __init__(self):
        self.throttle = 0.0
        self.steering = 0.0

    def set_throttle(self, val):
        print(f"[MOCK HW] set_throttle({val})")
        self.throttle = val

    def set_steering(self, val):
        self.steering = val


def test_watchdog():
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
