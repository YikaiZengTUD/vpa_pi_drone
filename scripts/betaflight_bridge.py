#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import rospy
import numpy as np
from protocol.h2rMultiWii import MultiWii
from protocol import arm_cmd,idle_cmd,disarm_cmd
from protocol import default_telemetry
from sensor_msgs.msg import Imu, BatteryState
from std_msgs.msg import Int32 
from serial import SerialException
from vpa_pi_drone.msg import RC
from tf import transformations
from enum import IntEnum


class DroneMode(IntEnum):
    DISARMED   = 0
    ARMED      = 1
    FLYING     = 2
    EMERGENCY  = 3


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
    /pidrone/mode
    """

    def __init__(self):
        self.board = self.getBoard()
        if not self.board:
            rospy.signal_shutdown("Failed to connect to flight controller")
            return
        rospy.init_node('betaflight_bridge', anonymous=False)
        


        self.prev_mode = DroneMode.DISARMED
        self.cur_mode  = DroneMode.DISARMED

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
        self.battery_msg = BatteryState()
        self.battery_pub = rospy.Publisher('pidrone/battery', BatteryState, queue_size=1)

        self.mode_sub  = rospy.Subscriber('pidrone/mode', Int32, self.mode_callback)
        self.send_fly_cmd = False
        self.fly_cmd_sub = rospy.Subscriber('pidrone/fly_commands', RC, self.fly_cmd_callback)

        # Register shutdown hook
        rospy.on_shutdown(self.safe_stop)

        rospy.loginfo("BetaflightBridge initialized, standing by")

    def mode_callback(self, msg):
        """Callback for mode changes."""
        new_mode = DroneMode(msg.data)
        if new_mode != self.cur_mode:
            rospy.loginfo(f"BetaflightBridge: Mode changed from {self.cur_mode.name} to {new_mode.name}")
            self.prev_mode = self.cur_mode
            self.cur_mode = new_mode

            self.handle_mode_transition()

    def fly_cmd_callback(self, msg):
        """Callback for flight commands."""
        self.last_command = self.command
        self.command = [
            int(msg.roll),
            int(msg.pitch),
            int(msg.yaw),
            int(msg.throttle),
            int(msg.aux1),
            int(msg.aux2),
            int(msg.aux3),
            int(msg.aux4)
        ]

    def safe_stop(self):
        """Safely stop the bridge by disarming the drone."""
        # TODO: change this to a more graceful land rather than just disarming and free falling
        rospy.loginfo("BetaflightBridge: Stopping safely")
        try:
            self.board.send_raw_command(8, MultiWii.SET_RAW_RC, disarm_cmd)
            rospy.loginfo("Disarm command sent successfully.")
        except Exception as e:
            rospy.logwarn(f"Disarm command failed: {e}")
        try:
            if self.board.ser.is_open:
                rospy.loginfo("Waiting for flight controller to respond to disarm command...")
                self.board.ser.timeout = 0.1  # Just in case
                # self.board.receiveDataPacket(terminated=True)
                rospy.loginfo("Flight controller responded to disarm command.")
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
        

    def get_battery_and_fillmsg(self):
        self.board.getData(MultiWii.ANALOG)
        self.battery_msg.header.stamp = rospy.Time.now()
        self.battery_msg.voltage  = self.board.analog['vbat'] * 0.1
        self.battery_msg.current  = self.board.analog['amperage']
        self.battery_msg.design_capacity = 1.5

    def handle_mode_transition(self):
        # send the command to the flight controller
        if self.cur_mode == DroneMode.DISARMED and (self.prev_mode == DroneMode.EMERGENCY or self.prev_mode == DroneMode.ARMED):
            self.board.send_raw_command(8, MultiWii.SET_RAW_RC, disarm_cmd)
            self.send_fly_cmd = False
            # emergency or armed to disarmed

        elif self.cur_mode == DroneMode.ARMED and self.prev_mode == DroneMode.DISARMED:
            rospy.loginfo("BetaflightBridge Sent: Switching to ARMED mode")
            self.board.send_raw_command(8, MultiWii.SET_RAW_RC, arm_cmd)
            self.command = arm_cmd
            self.send_fly_cmd = True
            # disarmed to armed
        
        elif self.cur_mode == DroneMode.FLYING and self.prev_mode == DroneMode.ARMED:
            self.send_fly_cmd = True
            self.command = idle_cmd
            # armed to flying
        elif self.cur_mode == DroneMode.DISARMED and self.prev_mode == DroneMode.FLYING:
            rospy.loginfo("BetaflightBridge Sent: Switching to DISARMED mode")
            self.send_fly_cmd = False
            self.board.send_raw_command(8, MultiWii.SET_RAW_RC, disarm_cmd)
            # flying to disarmed
        elif self.cur_mode == DroneMode.EMERGENCY and self.prev_mode == DroneMode.FLYING:
            rospy.loginfo("BetaflightBridge: Emergency mode activated, disarming, must restart")
            self.send_fly_cmd = False
            self.safe_stop()
            # flying to emergency

        elif self.cur_mode == DroneMode.ARMED and self.prev_mode == DroneMode.FLYING:
            # flying to armed
            pass
        else:
            rospy.logwarn(f"BetaflightBridge: Unhandled mode transition from {self.prev_mode.name} to {self.cur_mode.name}")


    def near_zero(self, value, threshold=0.001):
        return 0 if abs(value) < threshold else value

    def run(self):
        """Main loop of the bridge."""
        rate = rospy.Rate(60)
        try:
            while not rospy.is_shutdown():

                # Read the latest data from the flight controller
                if not hasattr(self, 'loop_counter'):
                    self.loop_counter = 0
                self.loop_counter += 1

                if self.loop_counter % 2 == 0:
                    self.get_imu_and_fillmsg()
                    self.imu_pub.publish(self.imu_msg)

                    self.get_battery_and_fillmsg()
                    self.battery_pub.publish(self.battery_msg)

                if self.send_fly_cmd:
                    self.board.send_raw_command(8, MultiWii.SET_RAW_RC, self.command)
                else:
                    self.board.send_raw_command(8, MultiWii.SET_RAW_RC, disarm_cmd)

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