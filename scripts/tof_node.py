#!/usr/bin/env python3

import rospy
import VL53L1X
import time
import sys
from sensor_msgs.msg import Range

class ToFVL53L1X(object):

    def __init__(self,address=0x29,bus_num=15):
        self.address = address
        self.bus_num = bus_num
        self.my_roi = VL53L1X.VL53L1xUserRoi(tlx=6, tly=6, brx=9, bry=9)
        self.tof = VL53L1X.VL53L1X(address=self.address, bus_num=self.bus_num)
        self.start_sensor()
        self.tof.set_timing(33000, 50)
        self.tof.set_distance_mode(2)
        self.tof.set_user_roi(self.my_roi)
        time.sleep(0.2)
        self.read_settings()

        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def timer_callback(self, event):
        try:
            distance = self.tof.get_distance()
            rospy.loginfo(f"Distance: {distance} mm")
            range_msg = Range()
            range_msg.header.stamp = rospy.Time.now()
            range_msg.header.frame_id = "bot_tof"
            range_msg.radiation_type = Range.INFRARED
            range_msg.field_of_view = 0.1
            range_msg.min_range = 0.04
            range_msg.max_range = 4.0
            range_msg.range = distance / 1000.0  # Convert mm to meters
            self.range_pub.publish(range_msg)
        except Exception as e:
            rospy.logerr(f"Error reading distance: {e}")

    def stop_sensor(self, *_):
        self.tof.stop_ranging()
        self.tof.close()
        rospy.loginfo("ToF sensor stopped cleanly.")
        sys.exit(0)

    def read_settings(self):
        timing_budget, inter_measurement = self.tof.get_timing()
        print("Timing budget (µs):", timing_budget)
        print("Inter-measurement period (ms):", inter_measurement)
        time.sleep(0.2)
        distance_mode = self.tof.get_distance_mode()
        print("Distance mode:", distance_mode)
        time.sleep(0.2)
        roi = self.tof.get_user_roi()
        print("Current ROI: tlx={}, tly={}, brx={}, bry={}".format(
            roi.tlx, roi.tly, roi.brx, roi.bry
        ))

    def start_sensor(self):
        time.sleep(0.2)
        self.tof.open()
        self.tof.stop_ranging()
        time.sleep(0.2)
        self.tof.start_ranging(0)
        rospy.loginfo(f"ToF sensor on bus {self.bus_num} initialized.")

if __name__ == '__main__':
    try:
        rospy.init_node('tof_node', anonymous=True)
        address = rospy.get_param('~address', 0x29)
        bus_num = rospy.get_param('~bus_num', 15)
        tof_sensor = ToFVL53L1X(address=address, bus_num=bus_num)
        tof_sensor.range_pub = rospy.Publisher('tof_range', Range, queue_size=10)
        rospy.on_shutdown(tof_sensor.stop_sensor)
        rospy.spin()
    except rospy.ROSInterruptException:
        pass