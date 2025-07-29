#!/usr/bin/env python3


import rospy
from sensor_msgs.msg import Joy
from vpa_pi_drone.msg import RC

from std_msgs.msg import Int32
from betaflight_bridge import DroneMode

'''
This script is designed to test the drone at early stage. It should convert the joystick input to RC commands and publish them.
The goal is to see the drone vertical movement and to test the drone's response to joystick inputs.

- Test the hover held throttle value
- Test the safe mode switch

'''

min_throttle = 1000
max_throttle = 2000

class JoyToRCConverter:
    def __init__(self):
        rospy.init_node('joy_to_rc_converter')
        
        self.joy_sub = rospy.Subscriber('joy', Joy, self.joy_callback)

        self.rc_pub  = rospy.Publisher('pidrone/fly_commands', RC, queue_size=1)
        self.safe_mode_pub = rospy.Publisher('pidrone/mode', Int32, queue_size=1)

        self.rc = RC()
        self.rc.yaw   = 1500
        self.rc.pitch = 1500
        self.rc.roll = 1500
        self.rc.throttle = 1000  # Set to hover throttle value
        self.rc.aux1 = 1800
        self.rc.aux2 = 1800
        self.rc.aux3 = 1000
        self.rc.aux4 = 1000

        self.mode = DroneMode.DISARMED

    def joy_callback(self, msg):
        rc_msg = RC()
        # Print detailed button states

        if msg.buttons[4] == 1:  
            self.rc.throttle = max(min_throttle, self.rc.throttle - 10)

        if msg.buttons[5] == 1:  
            if self.rc.throttle == min_throttle:
                self.rc.throttle = min(max_throttle, self.rc.throttle + 100)
            else:
                self.rc.throttle = min(max_throttle, self.rc.throttle + 10)

        print(f"Throttle: {self.rc.throttle}")

        if msg.buttons[3] == 1:  # Y
            self.mode = DroneMode.ARMED
            print("Switching to ARMED mode")
        if msg.buttons[0] == 1: # X
            self.mode = DroneMode.FLYING
            print("Switching to FLYING mode")
        if msg.buttons[1] == 1:  # B
            self.mode = DroneMode.DISARMED
            print("Switching to DISARMED mode")
        # Update the RC message
        
        if self.mode == DroneMode.FLYING:
            self.rc_pub.publish(self.rc)
 
        
        
        self.safe_mode_pub.publish(Int32(self.mode.value))

if __name__ == '__main__':
    try:
        converter = JoyToRCConverter()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass