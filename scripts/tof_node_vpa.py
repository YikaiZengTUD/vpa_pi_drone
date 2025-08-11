#!/usr/bin/env python3

from vpa_tof.VL53L1XPlus import VL53L1XPlus
import rospy
from sensor_msgs.msg import Range
from std_msgs.msg import Int32
import time
import sys

class ToFVL53L1XPlus:

    def __init__(self,address=0x29, bus_num=15):
        rospy.init_node('bot_tof', anonymous=False)
        rospy.on_shutdown(self.stop_sensor)
        self.sensor = VL53L1XPlus(i2c_address=address, i2c_bus=bus_num)

        self.sensor.open()
        self.sensor.start(mode='short', timing_budget_ms=33, intermeasurement_ms=50)

        self.range_pub = rospy.Publisher('bot_range', Range, queue_size=1)
        self.range_status_pub = rospy.Publisher('bot_range_status', Int32, queue_size=1)
        
        self.timer = rospy.Timer(rospy.Duration(0.1), self.timer_callback)

    def timer_callback(self, event):
        try:
            distance = self.sensor.get_distance_mm()
            status = self.sensor.get_range_status()
            range_msg = Range()
            range_msg.header.stamp = rospy.Time.now()
            range_msg.header.frame_id = "bot_tof"
            range_msg.radiation_type = Range.INFRARED
            range_msg.field_of_view = 0.1
            range_msg.min_range = 0.04
            range_msg.max_range = 4.0
            range_msg.range = distance / 1000.0  # Convert mm to meters
            self.range_pub.publish(range_msg)
            self.range_status_pub.publish(status)
        except Exception as e:
            rospy.logerr(f"Error reading distance: {e}")

    def stop_sensor(self, *_):
        self.sensor.stop()
        self.sensor.close()
        rospy.loginfo("ToF sensor stopped cleanly.")
        sys.exit(0)

if __name__ == '__main__':
    try:
        tof_sensor = ToFVL53L1XPlus()
        rospy.loginfo("ToF sensor initialized and running.")
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        tof_sensor.stop_sensor()