#!/usr/bin/env python3

import socket
import sys
import threading
import time
import io
import tf2_ros.transform_broadcaster

import rospy
from geometry_msgs.msg import Twist, PoseStamped
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion


def calculate_crc16(data):
    crc = 0xFFFF
    mask = 0xA001
    for d in data:
        crc = crc ^ d
        for _ in range(0, 8):
            if (crc & 0x0001) > 0:
                crc = (crc >> 1) ^ mask
            else:
                crc >>= 1
    return crc & 0xFFFF


def print_bytes(bs):
    res = ""
    for b in bs:
        res += "%02x " % b
    print(res)


class SensorHubClient:
    def __init__(self, host, port) -> None:
        self.__host = host
        self.__port = port
        self.__client_socket = None
        # self.__client_socket.settimeout(1)
        self.__has_connected = False
        self.__thread = threading.Thread(target=self.__server_thread)

    def run(self):
        rospy.init_node("sensor_hub_client")
        rospy.loginfo("start sensor hub client node.")

        self.__thread.start()
        rospy.spin()

    def __server_thread(self):
        while not rospy.is_shutdown():
            while not rospy.is_shutdown() and not self.__has_connected:
                try:
                    self.__client_socket = socket.socket(
                        socket.AF_INET, socket.SOCK_STREAM
                    )
                    rospy.logwarn_throttle(20, "try reconnect1")
                    self.__client_socket.connect((self.__host, self.__port))
                    self.__has_connected = True
                    rospy.loginfo("had connected to server")
                except Exception as _:
                    rospy.logwarn_throttle(20, "try reconnect2")
                    self.__has_connected = False
                    time.sleep(1)

            rospy.logwarn_throttle(20, "try reconnect3")

            rate = rospy.Rate(200)
            while not rospy.is_shutdown() and self.__has_connected:
                self.__receive_and_process()
                rate.sleep()

    # 接受控制指令，下发给主机
    def __receive_and_process(self):
        try:
            header1 = self.__client_socket.recv(1)
            header2 = self.__client_socket.recv(1)
            if header1 == b"\xba" and header2 == b"\xe2":
                self.__process_cmd_vel()
            elif header1 == b"\xba" and header2 == b"\xe3":
                self.__process_tf()
            elif header1 == b"\x50" and header2 == b"\x4f":
                self.__process_tracked_pose()
        except socket.timeout:
            rospy.logwarn("receive socket timeout")
        except Exception as e:
            rospy.logerr("socket disconnected %s" % str(e))
            self.__has_connected = False

    def __process_cmd_vel(self):
        rospy.loginfo_throttle(15, "found cmd vel data header")

        length = int.from_bytes(self.__client_socket.recv(4), byteorder="little")
        recv_crc = self.__client_socket.recv(2)
        payload = self.__client_socket.recv(length)
        calc_crc = calculate_crc16(payload).to_bytes(2, "little")
        if calc_crc != recv_crc:
            rospy.logerr_throttle(1, "cmd_vel crc error")

        twist = Twist()
        twist.deserialize(payload)
        rospy.logdebug(f"y is {twist.linear.x:.2f}, z is {twist.angular.z:.2f}")

    def __process_tf(self):
        rospy.loginfo_throttle(15, "found tf data header")

        length = int.from_bytes(self.__client_socket.recv(4), byteorder="little")
        recv_crc = self.__client_socket.recv(2)
        payload = self.__client_socket.recv(length)
        calc_crc = calculate_crc16(payload).to_bytes(2, "little")
        if calc_crc != recv_crc:
            rospy.logerr_throttle(1, "tf crc error")

        tf_msg = TFMessage()
        tf_msg.deserialize(payload)

    def __process_tracked_pose(self):
        rospy.loginfo_throttle(15, "found tracked pose header")

        length = int.from_bytes(self.__client_socket.recv(4), byteorder="little")
        recv_crc = self.__client_socket.recv(2)
        payload = self.__client_socket.recv(length)
        calc_crc = calculate_crc16(payload).to_bytes(2, "little")
        if calc_crc != recv_crc:
            rospy.logerr_throttle(1, "tf crc error")

        msg = PoseStamped()
        msg.deserialize(payload)

        quaternion = [
            msg.pose.orientation.x,
            msg.pose.orientation.y,
            msg.pose.orientation.z,
            msg.pose.orientation.w,
        ]

        x, y = msg.pose.position.x, msg.pose.position.y
        _, _, yaw = euler_from_quaternion(quaternion)
        rospy.loginfo_throttle(1, f"tracked pose is {x:03f}, {y:03f}, {yaw:03f}")


if __name__ == "__main__":
    address = "127.0.0.1"
    port = 8091

    if len(sys.argv) > 1:
        address = sys.argv[1]

    if len(sys.argv) > 2:
        port = int(sys.argv[2])

    client = SensorHubClient(address, port)
    client.run()
