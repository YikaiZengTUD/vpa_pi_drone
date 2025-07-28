#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from protocol.h2rMultiWii import MultiWii
from protocol import arm_cmd,idle_cmd,disarm_cmd
from protocol import default_telemetry
from sensor_msgs.msg import Imu, BatteryState
from serial import SerialException
# from vpsa_pi_drone.msg import RC
from tf import transformations
class BetaflightBridge:

    """A class that sends the current [r,p,y,t] commands to the flight
    controller board and then reads and publishes all of the data received
    from the flight controller.

    Publishers:
    /pidrone/imu
    /pidrone/battery
    /pidrone/mode

    Subscribers:
    /pidrone/fly_commands
    /pidrone/desired/mode
    /pidrone/heartbeat/range
    /pidrone/heartbeat/web_interface
    /pidrone/heartbeat/pid_controller
    /pidrone/state
    """

    def __init__(self):
        self.board = self.getBoard()
        if not self.board:
            rospy.signal_shutdown("Failed to connect to flight controller")
            return
        rospy.init_node('betaflight_bridge', anonymous=False)
        
        self.prev_mode = 'DISARMED'
        self.cur_mode  = 'DISARMED'

        self.command      = disarm_cmd
        self.last_command = self.command

        az = default_telemetry['az']
        self.accRawToMss = 9.8 / az if az != 0 else 1.0
        self.accZeroX = default_telemetry['ax'] * self.accRawToMss
        self.accZeroY = default_telemetry['ay'] * self.accRawToMss
        self.accZeroZ = default_telemetry['az'] * self.accRawToMss
        self.imu_msg = Imu()
        self.imu_pub = rospy.Publisher('pidrone/imu', Imu, queue_size=1)

        self.last_timestamp_imu = rospy.Time.now()

        self.last_timestamp_imu = rospy.Time.now()
    
        # Register shutdown hook
        rospy.on_shutdown(self.safe_stop)

        rospy.loginfo("BetaflightBridge initialized, standing by")

    def safe_stop(self):
        """Safely stop the bridge by disarming the drone."""
        # TODO: change this to a more graceful land rather than just disarming and free falling
        rospy.loginfo("BetaflightBridge: Stopping safely")
        try:
            self.board.send_raw_command(8, MultiWii.SET_RAW_RC, disarm_cmd)
        except Exception as e:
            rospy.logwarn(f"Disarm command failed: {e}")
        try:
            if self.board.ser.is_open:
                self.board.ser.timeout = 0.1  # Just in case
                self.board.receiveDataPacket()
        except Exception as e:
            rospy.logwarn(f"No response on shutdown (expected): {e}")

        try:
            self.board.close()
            rospy.loginfo("Serial port closed.")
        except Exception as e:
            rospy.logwarn(f"Failed to close serial port: {e}")


    def getBoard(self):
        try:
            board = MultiWii('/dev/ttyACM0')
        except Exception as e:
            rospy.logerr("ttyACM0 not found, trying /dev/ttyACM1")
            try:
                board = MultiWii('/dev/ttyACM1')
            except Exception as e:
                rospy.logerr("ttyACM1 not found, giving up")
                return None
        return board
    
    def get_imu_and_fillmsg(self):

        # extract roll, pitch, heading
        self.board.getData(MultiWii.ATTITUDE)
        # extract lin_acc_x, lin_acc_y, lin_acc_z
        self.board.getData(MultiWii.RAW_IMU)

        roll    = np.deg2rad(self.board.attitude['angx'])
        pitch   = np.deg2rad(self.board.attitude['angy'])
        heading = np.deg2rad(self.board.attitude['heading'])

        heading = (-heading) % (2 * np.pi)

        previous_quaternion = self.imu_msg.orientation
        
        previous_roll, previous_pitch, previous_heading = transformations.euler_from_quaternion(
            [previous_quaternion.x, previous_quaternion.y, previous_quaternion.z, previous_quaternion.w]
        )

        previous_heading = previous_heading % (2 * np.pi)

        quaternion = transformations.quaternion_from_euler(roll, pitch, heading)

        lin_acc_x = (self.board.rawIMU['ax'] * self.accRawToMss - self.accZeroX)
        lin_acc_y = (self.board.rawIMU['ay'] * self.accRawToMss - self.accZeroY)
        lin_acc_z = (self.board.rawIMU['az'] * self.accRawToMss - self.accZeroZ)

        lin_acc_x_drone_body = -lin_acc_y
        lin_acc_y_drone_body = lin_acc_x
        lin_acc_z_drone_body = lin_acc_z

        g = 9.8
        lin_acc_x_drone_body = lin_acc_x_drone_body + g*np.sin(roll)*np.cos(pitch)
        lin_acc_y_drone_body = lin_acc_y_drone_body + g*np.cos(roll)*(-np.sin(pitch))
        lin_acc_z_drone_body = lin_acc_z_drone_body + g*(1 - np.cos(roll)*np.cos(pitch))

        timestamp_imu = rospy.Time.now()
        dt = (timestamp_imu - self.last_timestamp_imu).to_sec()
        d_roll  = (roll - previous_roll) / dt
        d_pitch = (pitch - previous_pitch) / dt
        d_yaw   = (heading - previous_heading) / dt
        angvx   = self.near_zero(d_roll)
        angvy   = self.near_zero(d_pitch)
        angvz   = self.near_zero(d_yaw)
        self.last_timestamp_imu = timestamp_imu

        self.imu_msg.header.stamp = timestamp_imu
        self.imu_msg.header.frame_id = 'drone_frame' # already converted from IMU frame
        self.imu_msg.orientation.x = quaternion[0]
        self.imu_msg.orientation.y = quaternion[1]
        self.imu_msg.orientation.z = quaternion[2]
        self.imu_msg.orientation.w = quaternion[3]
        self.imu_msg.angular_velocity.x = angvx
        self.imu_msg.angular_velocity.y = angvy
        self.imu_msg.angular_velocity.z = angvz
        self.imu_msg.linear_acceleration.x = lin_acc_x_drone_body
        self.imu_msg.linear_acceleration.y = lin_acc_y_drone_body
        self.imu_msg.linear_acceleration.z = lin_acc_z_drone_body


    def near_zero(self, value, threshold=0.001):
        return 0 if abs(value) < threshold else value

    def run(self):
        """Main loop of the bridge."""
        rate = rospy.Rate(30)
        try:
            while not rospy.is_shutdown():

                self.get_imu_and_fillmsg()
                self.imu_pub.publish(self.imu_msg)
                # print("IMU data published")
                # print("Roll: {:.2f}, Pitch: {:.2f}, Heading: {:.2f}".format(
                #     np.rad2deg(self.imu_msg.orientation.x),
                #     np.rad2deg(self.imu_msg.orientation.y),
                #     np.rad2deg(self.imu_msg.orientation.z)
                # ))
                rate.sleep()

        except SerialException as e:
            rospy.logerr("SerialException: %s", e)
            rospy.signal_shutdown("SerialException occurred")
        except Exception as e:
            rospy.logerr("Exception: %s", e)
            rospy.signal_shutdown("Exception occurred")
        finally:
            rospy.loginfo("BetaflightBridge shutting down")
            rospy.signal_shutdown("Exception occurred")
            
if __name__ == '__main__':
    try:
        bridge = BetaflightBridge()
        bridge.run()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        rospy.loginfo("KeyboardInterrupt: Shutting down BetaflightBridge")