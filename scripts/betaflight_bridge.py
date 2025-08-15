#!/usr/bin/env python3

from protocol.MultiWiiPY3 import MultiWii
from protocol import arm_cmd, disarm_cmd, idle_cmd, fail_cmd
import enum 
import time
class BetaFlightController:
    # This is the controller itself, no need to do ROS wrap now, will do it with another class here later

    class DroneState(enum.Enum):
        DISARMED    = 0
        ARMED       = 1
        FLYING      = 2
        FAILING     = 3 # this is a state we go low on throttle and try land softly instead of direct disarm

    def __init__(self, serial_port="/dev/ttyACM0"):
        self.Board = MultiWii(serial_port)


    def arm(self):
        self.Board.send_raw_rc(16, MultiWii.SET_RAW_RC, arm_cmd)
        ack = self.Board.receiveDataPacket()
        return ack

    def disarm(self):
        self.Board.send_raw_rc(16, MultiWii.SET_RAW_RC, disarm_cmd)
        ack = self.Board.receiveDataPacket()
        return ack

    def idle(self):
        self.Board.send_raw_rc(16, MultiWii.SET_RAW_RC, idle_cmd)
        ack = self.Board.receiveDataPacket()
        return ack
    
    def fail(self):
        self.Board.send_raw_rc(16, MultiWii.SET_RAW_RC, fail_cmd)
        ack = self.Board.receiveDataPacket()
        return ack

    def _check_age(self,readtime,timestamp):
        age = readtime - timestamp
        return age < 0 # this is new message after requesting

    def send_flying_command(self, command:list):
        """
        Sends a flying command to the drone.
        :param command: A list of control values for the drone.
        """
        if len(command) != 8:
            print("Command must be a list of 8 values.")
            return None

        # Ensure all values are integers (cast from float if needed)
        command = [int(x) for x in command]

        self.Board.send_raw_rc(16, MultiWii.SET_RAW_RC, command)
        ack = self.Board.receiveDataPacket()
        return ack

    def requesting_reading_imu(self):
        readtime = time.time()
        self.Board.getData(MultiWii.RAW_IMU)
        return self._check_age(readtime, self.Board.rawIMU['timestamp'])

    def requesting_reading_attitude(self):
        readtime = time.time()
        self.Board.getData(MultiWii.ATTITUDE)
        return self._check_age(readtime, self.Board.attitude['timestamp'])

    def requesting_reading_battery(self):
        readtime = time.time()
        self.Board.getData(MultiWii.ANALOG)
        return self._check_age(readtime, self.Board.analog['timestamp'])

    def close(self):
        self.Board.close()

import rospy
from sensor_msgs.msg import Imu, BatteryState
from std_msgs.msg import Int32
from vpa_pi_drone.msg import RC
import numpy as np
import tf.transformations as tf_trans
class BetaFlightROSBridge:

    ACC_SCALE = 9.80665 / 512.0  # m/s^2 per raw unit (1g = 512)
    GYRO_SCALE = 1.0 / 16.4  # deg/s per raw unit (MPU-6000, ±2000 deg/s)
    DEG2RAD = np.pi / 180.0

    def __init__(self):
        rospy.init_node('MSP_bridge')
        rospy.on_shutdown(self.close)
        self.imu_pub = rospy.Publisher('imu', Imu, queue_size=1)
        self.att_pub = rospy.Publisher('attitude', Imu, queue_size=1)
        self.battery_pub = rospy.Publisher('battery', BatteryState, queue_size=10)
        self.drone_state = BetaFlightController.DroneState.DISARMED
        self.drone_state_sub = rospy.Subscriber('drone_state', Int32, self.state_cb)
        self.controller = BetaFlightController()
        self.rate = rospy.Rate(50)  # 50 Hz
        self.flying_command = idle_cmd  # Default command is idle
        self.flying_command_sub = rospy.Subscriber('flying_command', RC, self.flying_command_cb)


    def fill_IMUROS_msg(self):
        msg = Imu()
        # Timestamp and frame
        timestamp = self.controller.Board.rawIMU['timestamp']
        msg.header.stamp = rospy.Time.from_sec(timestamp)
        msg.header.frame_id = "imu_frame"

        # Linear acceleration (m/s^2)
        msg.linear_acceleration.x = self.controller.Board.rawIMU['ax'] * self.ACC_SCALE
        msg.linear_acceleration.y = self.controller.Board.rawIMU['ay'] * self.ACC_SCALE
        msg.linear_acceleration.z = self.controller.Board.rawIMU['az'] * self.ACC_SCALE

        # Angular velocity (from gyro, raw to rad/s)

        msg.angular_velocity.x = self.controller.Board.rawIMU['gx'] * self.GYRO_SCALE * self.DEG2RAD
        msg.angular_velocity.y = self.controller.Board.rawIMU['gy'] * self.GYRO_SCALE * self.DEG2RAD
        msg.angular_velocity.z = self.controller.Board.rawIMU['gz'] * self.GYRO_SCALE * self.DEG2RAD
        return msg

    def fill_AttitudeROS_msg(self):
        msg = Imu()
        # Timestamp and frame
        timestamp = self.controller.Board.attitude['timestamp']
        msg.header.stamp = rospy.Time.from_sec(timestamp)
        msg.header.frame_id = "horizontal_drone_frame"

        # Orientation (from gyro, raw to rad)
        roll = self.controller.Board.attitude['angx'] * self.DEG2RAD
        pitch = -self.controller.Board.attitude['angy'] * self.DEG2RAD
        yaw = self.controller.Board.attitude['heading'] * self.DEG2RAD
        quat = tf_trans.quaternion_from_euler(roll, pitch, yaw)
        msg.orientation.x = quat[0]
        msg.orientation.y = quat[1]
        msg.orientation.z = quat[2]
        msg.orientation.w = quat[3]
        return msg

    def fill_BatteryROS_msg(self):
        msg = BatteryState()
        # Timestamp and frame
        timestamp = self.controller.Board.analog['timestamp']
        msg.header.stamp = rospy.Time.from_sec(timestamp)
    
        # Battery voltage and current
        msg.voltage = self.controller.Board.analog['vbat'] * 0.1  # Convert mV to V
        msg.current = self.controller.Board.analog['amperage'] # Convert mA to A
        # so far cant inferr the other information 
        return msg

    def state_cb(self, msg):
        if self.drone_state != BetaFlightController.DroneState(msg.data):
            rospy.loginfo(f"Drone state changed to: {BetaFlightController.DroneState(msg.data).name}")
            self.drone_state = BetaFlightController.DroneState(msg.data)

    def flying_command_cb(self, msg:RC):
        self.flying_command = [msg.roll, msg.pitch, msg.yaw, msg.throttle, msg.aux1, msg.aux2, 1000, 1000] #  aux3,4 not in use

    def run(self):
        READ_IMU = True
        while not rospy.is_shutdown():

            if self.drone_state == BetaFlightController.DroneState.ARMED:
                ack = self.controller.arm()
            elif self.drone_state == BetaFlightController.DroneState.FAILING:
                ack = self.controller.fail()
            elif self.drone_state == BetaFlightController.DroneState.FLYING:
                ack = self.controller.send_flying_command(self.flying_command)  
            else:
                ack = self.controller.disarm()  # always sending this
            
            # reading is not need for that frequent
            if READ_IMU:
                try:
                    imu_msg_new = self.controller.requesting_reading_imu()
                    att_msg_new = self.controller.requesting_reading_attitude()
                    if imu_msg_new and att_msg_new:
                        self.imu_pub.publish(self.fill_IMUROS_msg())
                        self.att_pub.publish(self.fill_AttitudeROS_msg())
                except Exception as e:
                    rospy.logerr(f"Error reading IMU/Attitude: {e}")
                
            else:
                try:  
                    battery_msg_new = self.controller.requesting_reading_battery()
                    if battery_msg_new:
                        self.battery_pub.publish(self.fill_BatteryROS_msg())
                except Exception as e:
                    rospy.logerr(f"Error reading Battery: {e}")

            READ_IMU = not READ_IMU
            self.rate.sleep()

    def close(self):
        rospy.loginfo("BetaFlightROSBridge close() called.")
        self.controller.close()
        rospy.loginfo("BetaFlightROSBridge closed.")

if __name__ == '__main__':
    bridge = BetaFlightROSBridge()
    try:
        bridge.run()
    except rospy.ROSInterruptException:
        print("ROS interrupt exception, shutting down.")
        pass
    except KeyboardInterrupt:
        rospy.loginfo("Keyboard interrupt received, shutting down.")
    finally:
        bridge.close()
