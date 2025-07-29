#!/usr/bin/env python3

import time
from protocol.h2rMultiWii import MultiWii
import struct
SERIAL_PORT = "/dev/ttyACM0"  # Replace with correct port if needed

# Predefined RC values
# Channels: [Roll, Pitch, Yaw, Throttle, AUX1 (ARM), AUX2 (ANGLE), AUX3, AUX4]
ARM_CMD    = [1500, 1500, 1500, 1000, 1800, 1000, 1000, 1000]
DISARM_CMD = [1500, 1500, 1500, 1000, 1000, 1000, 1000, 1000]

def send_rc_for_duration(board, cmd, duration_sec, label=""):
    print(f"Sending {label} command for {duration_sec} seconds...")
    t0 = time.time()
    while time.time() - t0 < duration_sec:
        board.send_raw_command(8, MultiWii.SET_RAW_RC, cmd)
        time.sleep(0.0167)  # ~60 Hz

def get_arming_flags(board):
    MSP_STATUS_EX = 101
    board.send_raw_command(0, MSP_STATUS_EX, [])
    print("Requesting arming flags...")
    response = board.receiveDataPacket()
    print("Received arming flags response:", response)
    if response is None:
        print("No response from FC")
        return

    # Unpack first 4 bytes as uint32 = armingDisableFlags
   

    # Optional: interpret known bits
    # reasons = {
    #     0: "THROTTLE",
    #     1: "ANGLE",
    #     2: "BOOT_GRACE_TIME",
    #     3: "NO_GYRO",
    #     4: "WAITING_FOR_RX",
    #     5: "FAILSAFE",
    #     6: "RX_FAILSAFE",
    #     7: "RUNAWAY_TAKEOFF",
    #     8: "CRASH_DETECTED",
    #     9: "BAT_LOW",
    #     10: "MSP",
    #     11: "SENSOR_CALIBRATION",
    #     # add more as needed
    # }
    # for bit, reason in reasons.items():
    #     if flags & (1 << bit):
    #         print(f"→ Arming blocked by: {reason}")

if __name__ == '__main__':
    print("Connecting to flight controller...")
    board = MultiWii(SERIAL_PORT)
    board.ser.timeout = 1
    try:
        send_rc_for_duration(board, DISARM_CMD, 2, "DISARM")
        send_rc_for_duration(board, ARM_CMD, 3, "ARM")
        get_arming_flags(board)

        send_rc_for_duration(board, DISARM_CMD, 2, "DISARM")
    finally:
        board.close()
        print("Test complete.")
