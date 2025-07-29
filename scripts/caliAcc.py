#!/usr/bin/env python3

from protocol.h2rMultiWii import MultiWii
import time
import rospy
import sys
import numpy as np
def main():
    try:
        board = MultiWii("/dev/ttyACM0")
        rospy.loginfo("Calibration ready, the drone is level and still?")
        while True:
            answer = input("Enter Y/N: ").upper()
            if answer in ['Y', 'N']:
                break
        if answer == 'N':
            print("Please level the drone and try again.")
            sys.exit(0)
        board.send_raw_command(0, MultiWii.ACC_CALIBRATION, [])
        board.receiveDataPacket()
        time.sleep(2)
        print("Calibration complete")
        board.getData(MultiWii.ATTITUDE)
        board.getData(MultiWii.RAW_IMU)
        roll    = np.deg2rad(board.attitude['angx'])
        pitch   = np.deg2rad(board.attitude['angy'])
        heading = np.deg2rad(board.attitude['heading'])
        print(f"Roll: {roll}, Pitch: {pitch}, Heading: {heading} (degrees)")
        raw_acc_x = board.rawIMU['ax']
        raw_acc_y = board.rawIMU['ay']
        raw_acc_z = board.rawIMU['az']

        drone_acc_x = - raw_acc_y
        drone_acc_y = raw_acc_x
        drone_acc_z = raw_acc_z

        print(f"Drone Acceleration - X: {drone_acc_x}, Y: {drone_acc_y}, Z: {drone_acc_z} (raw values)")

    except Exception as e:
        print(f"Error connecting to MultiWii board: {e}")
        return