#! /usr/bin/env python3
# -*- coding: utf-8 -*-


import rospy
from sensor_msgs.msg import Joy, Range
from vpa_pi_drone.msg import RC
from std_msgs.msg import Int32
from betaflight_bridge import BetaFlightController
from dynamic_reconfigure.server import Server as DynamicReconfigureServer
from vpa_pi_drone.cfg import HeightPIDConfig

DroneMode = BetaFlightController.DroneState

min_throttle = 1000
max_throttle = 2000


class PIDController:
    def __init__(self, kp=8, ki=1.0, kd=0.0, base=1515, integral_limit=3.0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.base = base
        self.integral_limit = integral_limit
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = rospy.get_time()

    def reset(self):
        self.integral = 0.0
        self.last_error = 0.0
        self.last_time = rospy.get_time()

    def update(self, error):
        now = rospy.get_time()
        dt = now - self.last_time if self.last_time else 0.05
        self.last_time = now
        # Integral with anti-windup
        self.integral += error * dt
        self.integral = max(-self.integral_limit, min(self.integral_limit, self.integral))
        # Derivative
        derivative = (error - self.last_error) / dt if dt > 0 else 0.0
        self.last_error = error
        # PID output
        output = self.base + self.kp * error + self.ki * self.integral + self.kd * derivative
        return output, self.integral


class JoyToFHController:

    def __init__(self):
        rospy.init_node('joy_to_fh_controller')

        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)
        self.height  = 0
        self.height_sub = rospy.Subscriber('bot_range', Range, self.height_callback)

        self.rc_pub = rospy.Publisher('flying_command', RC, queue_size=1)
        self.safe_mode_pub = rospy.Publisher('drone_state', Int32, queue_size=1)

        self.rc = RC()
        self.rc.yaw   = 1500
        self.rc.pitch = 1500
        self.rc.roll  = 1500
        self.rc.throttle = 1100

        self.rc.aux1 = 1800
        self.rc.aux2 = 1800
        self.rc.aux3 = 1000
        self.rc.aux4 = 1000

        self.height_ref = 0.6
        self.pid_output = False

    # Initial PID values, will be overwritten by dynamic reconfigure
        self.pid = PIDController()
        self.mode = DroneMode.DISARMED
        rospy.loginfo("Joy to FH Controller initialized")
        # Dynamic reconfigure server
        self.dyn_server = DynamicReconfigureServer(HeightPIDConfig, self.reconfig_callback)
        self.timer = rospy.Timer(rospy.Duration(0.05), self.pid_control)

    def reconfig_callback(self, config, level):
        self.pid.kp = config.kp
        self.pid.ki = config.ki
        self.pid.kd = config.kd
        self.pid.base = config.base_throttle
        self.pid.integral_limit = config.integral_limit
        self.height_ref = config.height_ref
        return config

    def joy_callback(self, msg):
        
        if msg.buttons[3] == 1:  # Y
            self.mode = DroneMode.ARMED
            print("Switching to ARMED mode")
            self.pid_output = False
        if msg.buttons[0] == 1: # X
            self.mode = DroneMode.FLYING
            self.pid_output = True
            print("Switching to FLYING mode - This mode goes to 0.6m (hardcoded)")
        if msg.buttons[1] == 1:  # B
            self.mode = DroneMode.DISARMED
            self.pid_output = False
            print("Switching to DISARMED mode")
        if msg.buttons[2] == 1: # A
            print('Low Throttle descend')
            self.pid_output = False
            self.mode = DroneMode.FAILING

        self.safe_mode_pub.publish(Int32(data=self.mode.value))  # Notify the system that 

    def height_callback(self, msg):
        self.height = max(0.0, msg.range - 0.05)

    def pid_control(self, event=None):
        if self.pid_output:
            error = self.height_ref - self.height
            # Deadband for small errors
            if abs(error) < 0.05:
                error = 0.0
            throttle, integral = self.pid.update(error)
            self.rc.throttle = int(max(min_throttle, min(max_throttle, throttle)))
            self.rc_pub.publish(self.rc)
            # rospy.loginfo(f"Height: {self.height:.2f}, Error: {error:.2f}, Throttle: {self.rc.throttle}, Integral: {integral:.2f}")
        else:
            self.rc.throttle = 1400
            self.rc_pub.publish(self.rc)

if __name__ == '__main__':
    controller = JoyToFHController()
    rospy.spin()
