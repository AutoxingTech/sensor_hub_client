#!/usr/bin/env python3

import socket
import sys
import threading
import time
import io
import tf2_ros.transform_broadcaster

import rospy
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu

from rplidar_ros.msg import AxLaserScan
from sensor_hub_client.msg import TcpRobotState
from sensor_hub_client.msg import TcpRobotControl

from cln_msgs.msg import HardwareCtrl
from cln_msgs.msg import SensorType
from cln_msgs.msg import CleanDeviceType
from cln_msgs.msg import HardwareState
from cln_msgs.msg import HardwareStateType


# 此程序部署到要移植的机器中，收发宿主机的传感器数据，并通过 tcp 传输到 docker 内部

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
        self.__cmd_vel_publisher = None
        self.__control_publisher = None
        self.__hardware_state_index = 0 # 用于降采样
        self.__tf2_broadcaster = tf2_ros.transform_broadcaster.TransformBroadcaster()
        
        self.__last_imu_time = rospy.Time(0)
        self.__last_lidar_time = rospy.Time(0)
        self.__last_odom_time = rospy.Time(0)
        
        self.__thread = threading.Thread(target=self.__server_thread)


    def run(self):
        rospy.init_node("sensor_hub_client")
        rospy.loginfo("start sensor hub client node.")

        self.__cmd_vel_publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=20)
        self.__control_publisher = rospy.Publisher("/automode_ctrl", HardwareCtrl, queue_size=10)

        # todo: 通过参数来设置 topic name
        rospy.Subscriber("/imu", Imu, self.__imu_callback)
        rospy.Subscriber("/odom", Odometry, self.__odometry_callback)
        rospy.Subscriber("/hardware_state", HardwareState, self.__hardware_state_callback)
        rospy.Subscriber("/ax_laser_scan", AxLaserScan, self.__laser_scan_callback)

        self.__thread.start()
        rospy.spin()


    def __server_thread(self):
        while not rospy.is_shutdown():
            while not rospy.is_shutdown() and not self.__has_connected:
                try:
                    self.__client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
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


    def __imu_callback(self, msg: Imu):
        if not self.__has_connected:
            rospy.logwarn_throttle(20, "Receive Imu callback, but us dose not connect")
            return
        
        assert(self.__last_imu_time < msg.header.stamp)
        self.__last_imu_time = msg.header.stamp

        rospy.loginfo_throttle(15, "__imu_callback")
        buff = io.BytesIO()
        msg.serialize(buff)
        self.__send_to_server(b"\xAB\xCD", buff.getvalue())


    def __odometry_callback(self, msg: Odometry):
        if not self.__has_connected:
            rospy.logwarn_throttle(20, "Receive Odometry callback, but us dose not connect")
            return

        assert(self.__last_odom_time < msg.header.stamp)
        self.__last_odom_time = msg.header.stamp

        rospy.loginfo_throttle(15, "__odometry_callback")
        buff = io.BytesIO()
        msg.serialize(buff)
        self.__send_to_server(b"\xAB\xCE", buff.getvalue())


    def __laser_scan_callback(self, msg: AxLaserScan):
        if not self.__has_connected:
            rospy.logwarn_throttle(20, "Receive AxLaserScan callback, but us dose not connect")
            return

        assert(self.__last_lidar_time < msg.header.stamp)
        self.__last_lidar_time = msg.header.stamp

        buff = io.BytesIO()
        msg.serialize(buff)
        rospy.loginfo_throttle(15, f"__laser_scan_callback: {len(buff.getvalue())}")
        self.__send_to_server(b"\xAB\xCF", buff.getvalue())


    def __hardware_state_callback(self, msg: HardwareState):
        if not self.__has_connected:
            rospy.logwarn_throttle(20, "Receive HardwareState callback, but us dose not connect")
            return

        self.__hardware_state_index += 1
        if self.__hardware_state_index % 5 != 0:
            return

        state = TcpRobotState()
        # msg.manual: True 代表无动力； False 代表机器驱动
        if msg.manual:
            state.wheels_enabled = False
        else:
            state.wheels_enabled = True
        
        state.is_charge = msg.bat_state # 0 放电 1 充电
        state.battery_percent = msg.bat_percentage # 百分比 0~100
        rospy.logdebug_throttle(10, f"bat_percentage: {msg.bat_percentage}")

        buff = io.BytesIO()
        state.serialize(buff)
        self.__send_to_server(b"\xAB\xD0", buff.getvalue())


    # header + length + crc(body) + body
    def __send_to_server(self, header, body,):
        try:
            data = header
            data += len(body).to_bytes(4, "little")
            data += calculate_crc16(body).to_bytes(2, "little")
            data += body
            self.__client_socket.send(data)
        except Exception as e:
            self.__has_connected = False
            rospy.logerr("socket disconnected %s" % str(e))


    # 接受控制指令，下发给主机
    def __receive_and_process(self):
        try:
            if self.__client_socket.recv(1) != b"\xba":
                return

            header2 = self.__client_socket.recv(1)
            if header2 == b"\xe1":
                self.__control_wheel()
            elif header2 == b"\xe2":
                self.__process_cmd_vel()
            elif header2 == b"\xe3":
                self.__process_tf()
        except socket.timeout:
            rospy.logwarn("receive socket timeout")
        except Exception as e:
            rospy.logerr("socket disconnected %s" % str(e))
            self.__has_connected = False


    def __control_wheel(self):
        rospy.loginfo_throttle(15, "found control wheel data header")

        length = int.from_bytes(self.__client_socket.recv(4), byteorder='little')
        recv_crc = self.__client_socket.recv(2)
        payload = self.__client_socket.recv(length)
        calc_crc = calculate_crc16(payload).to_bytes(2, "little")
        if calc_crc != recv_crc:
            rospy.logerr_throttle(1, "control wheel data crc error")

        robotControl = TcpRobotControl()
        robotControl.deserialize(payload)
        rospy.loginfo(f"robotControl.control_mode is {robotControl.enable_wheels}")
        
        ctrl = HardwareCtrl()
        ctrl.sensor_id.append(SensorType(0))
        ctrl.device_id.append(CleanDeviceType(0))

        if robotControl.enable_wheels:
            # 自动模式
            ctrl.state.append(HardwareStateType(7))
            rospy.loginfo("should switch to auto mode")
        else:
            # 手动模式
            ctrl.state.append(HardwareStateType(8))
            rospy.loginfo("should switch to manual mode")

        self.__control_publisher.publish(ctrl)


    def __process_cmd_vel(self):
        rospy.loginfo_throttle(15, "found cmd vel data header")

        length = int.from_bytes(self.__client_socket.recv(4), byteorder='little')
        recv_crc = self.__client_socket.recv(2)
        payload = self.__client_socket.recv(length)
        calc_crc = calculate_crc16(payload).to_bytes(2, "little")
        if calc_crc != recv_crc:
            rospy.logerr_throttle(1, "cmd_vel crc error")

        twist = Twist()
        twist.deserialize(payload)
        rospy.logdebug(f"y is {twist.linear.x:.2f}, z is {twist.angular.z:.2f}")
        self.__cmd_vel_publisher.publish(twist)


    def __process_tf(self):
        rospy.loginfo_throttle(15, "found tf data header")

        length = int.from_bytes(self.__client_socket.recv(4), byteorder='little')
        recv_crc = self.__client_socket.recv(2)
        payload = self.__client_socket.recv(length)
        calc_crc = calculate_crc16(payload).to_bytes(2, "little")
        if calc_crc != recv_crc:
            rospy.logerr_throttle(1, "cmd_vel crc error")

        tf_msg = TFMessage()
        tf_msg.deserialize(payload)
        self.__tf2_broadcaster.sendTransform(tf_msg.transforms)


if __name__ == "__main__":
    address = "0.0.0.0"
    port = 8091

    if len(sys.argv) > 1:
        address = sys.argv[1]

    if len(sys.argv) > 2:
        port = int(sys.argv[2])

    client = SensorHubClient(address, port)
    client.run()
