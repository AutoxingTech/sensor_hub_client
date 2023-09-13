#!/usr/bin/env python3

import rospy
from rplidar_ros.msg import AxLaserScan

class SensorHubDebugger:
    def __init__(self) -> None:
        pass

    def run(self):
        rospy.init_node("sensor_hub_debugger")
        rospy.Subscriber("/ax_laser_scan", AxLaserScan, self.__ax_laser_scan_callback)
        rospy.spin()

    def __ax_laser_scan_callback(self, msg: AxLaserScan):
        angle_sum = 0
        
        factor = 360. / 65536.
        min_angle = (msg.angles[0] * factor)
        max_angle = (msg.angles[-1] * factor)
        print(f"head angle is {min_angle}, tail angle is {max_angle}")
        
        last_angle = (msg.angles[0] * factor)
        for angle in msg.angles:
            cur_angle = (angle * factor)
            delta = cur_angle - last_angle
            if cur_angle < last_angle:
                delta += 360.
                
            angle_sum += delta
            last_angle = cur_angle
            
        print(f"angle_sum is {angle_sum}")


if __name__ == "__main__":
    debugger = SensorHubDebugger()
    debugger.run()