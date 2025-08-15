#!/usr/bin/env python3
# what to expect
# this node will send commands to the BetaFlightROSBridge
# 1. hold for 10sec, not sending anything
# 2. send arm command
# 3. send flying command, and bt 50 as step raise the throttle to 1250, 10 sec hold each stage
# 4. reduce back by step same time
# 5. disarm

import rospy


from std_msgs.msg import Int32
from vpa_pi_drone.msg import RC
import time

class bridge_tester:
    def __init__(self):
        rospy.init_node('msp_bridge_tester')
        self.state_pub = rospy.Publisher('drone_state', Int32, queue_size=1)
        self.cmd_pub = rospy.Publisher('flying_command', RC, queue_size=1)
        self.rate = rospy.Rate(10)  # 10 Hz

    def publish_state(self, state):
        msg = Int32()
        msg.data = state
        self.state_pub.publish(msg)

    def publish_cmd(self, roll=1500, pitch=1500, yaw=1500, throttle=1000, aux1=1800, aux2=1800, aux3=1000, aux4=1000):
        msg = RC()
        msg.roll = roll
        msg.pitch = pitch
        msg.yaw = yaw
        msg.throttle = throttle
        msg.aux1 = aux1
        msg.aux2 = aux2
        msg.aux3 = aux3
        msg.aux4 = aux4
        self.cmd_pub.publish(msg)

    def run(self):
        # 1. Hold for 10 sec, not sending anything
        rospy.loginfo("[TEST] Holding for 10 seconds...")
        time.sleep(10)

        # 2. Send arm command
        rospy.loginfo("[TEST] Sending ARM command...")
        self.publish_state(1)  # ARMED
        self.publish_cmd()     # Default RC values
        time.sleep(2)

        # 3. Send flying command, ramp throttle up
        throttle = 1000
        while throttle <= 1150:
            rospy.loginfo(f"[TEST] Throttle: {throttle}")
            self.publish_state(2)  # FLYING
            time.sleep(0.2)
            self.publish_cmd(throttle=throttle, aux1=1800)  # aux1 high for ARM
            time.sleep(8)
            throttle += 50

        # 4. Reduce throttle back down
        throttle = 1150
        while throttle >= 1000:
            rospy.loginfo(f"[TEST] Throttle: {throttle}")
            self.publish_state(2)  # FLYING
            self.publish_cmd(throttle=throttle, aux1=1800)
            time.sleep(10)
            throttle -= 50

        # 5. Disarm
        rospy.loginfo("[TEST] Sending DISARM command...")
        self.publish_state(0)  # DISARMED
        self.publish_cmd()
        time.sleep(2)

        rospy.loginfo("[TEST] Test sequence complete.")


if __name__ == "__main__":
    tester = bridge_tester()
    try:
        tester.run()
    except rospy.ROSInterruptException:
        pass
