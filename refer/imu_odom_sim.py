#!/usr/bin/env python3
"""
IMU and Odometry Simulator Client
Sends fake IMU data at 50 Hz and fake odometry data at 20 Hz to the sensor hub server.
"""

import socket
import sys
import time
import threading
import math

import rospy
from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Quaternion


def calculate_crc16(data):
    """Calculate CRC16 checksum"""
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


class ImuOdomSimulator:
    def __init__(self, host, port):
        self.__host = host
        self.__port = port
        self.__client_socket = None
        self.__has_connected = False
        self.__running = False
        
        # Thread for connection management
        self.__connect_thread = threading.Thread(target=self.__connection_thread)
        # Thread for IMU data (50 Hz)
        self.__imu_thread = threading.Thread(target=self.__imu_send_thread)
        # Thread for Odometry data (20 Hz)
        self.__odom_thread = threading.Thread(target=self.__odom_send_thread)

    def run(self):
        """Start the simulator"""
        rospy.init_node("imu_odom_simulator")
        rospy.loginfo("Starting IMU and Odometry simulator")
        
        self.__running = True
        self.__connect_thread.start()
        self.__imu_thread.start()
        self.__odom_thread.start()
        
        rospy.spin()
        
        # Cleanup
        self.__running = False
        if self.__client_socket:
            self.__client_socket.close()

    def __connection_thread(self):
        """Manages connection to the server"""
        while not rospy.is_shutdown() and self.__running:
            if not self.__has_connected:
                try:
                    self.__client_socket = socket.socket(
                        socket.AF_INET, socket.SOCK_STREAM
                    )
                    self.__client_socket.connect((self.__host, self.__port))
                    self.__has_connected = True
                    rospy.loginfo(f"Connected to server at {self.__host}:{self.__port}")
                except Exception as e:
                    rospy.logwarn_throttle(5, f"Failed to connect: {e}. Retrying...")
                    self.__has_connected = False
                    if self.__client_socket:
                        self.__client_socket.close()
                    time.sleep(1)
            else:
                time.sleep(1)

    def __send_packet(self, header, payload):
        """Send a packet with header, length, CRC, and payload"""
        if not self.__has_connected or not self.__client_socket:
            return False
        
        try:
            length = len(payload)
            crc = calculate_crc16(payload)
            
            # Pack the message: header(2) + length(4) + crc(2) + payload
            packet = bytearray()
            packet.extend(header)
            packet.extend(length.to_bytes(4, byteorder='little'))
            packet.extend(crc.to_bytes(2, byteorder='little'))
            packet.extend(payload)
            
            self.__client_socket.sendall(packet)
            return True
        except Exception as e:
            rospy.logerr(f"Send error: {e}")
            self.__has_connected = False
            if self.__client_socket:
                self.__client_socket.close()
            return False

    def __create_fake_imu_msg(self):
        """Create a fake IMU message with some simulated motion"""
        msg = Imu()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "imu_link"
        
        # Fake orientation (quaternion for slight rotation over time)
        t = rospy.Time.now().to_sec()
        angle = math.sin(t * 0.5) * 0.1  # Small oscillation
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = math.sin(angle / 2.0)
        msg.orientation.w = math.cos(angle / 2.0)
        
        # Fake angular velocity
        msg.angular_velocity.x = math.sin(t) * 0.01
        msg.angular_velocity.y = math.cos(t) * 0.01
        msg.angular_velocity.z = math.sin(t * 0.5) * 0.05
        
        # Fake linear acceleration (gravity + some noise)
        msg.linear_acceleration.x = math.sin(t * 2) * 0.1
        msg.linear_acceleration.y = math.cos(t * 2) * 0.1
        msg.linear_acceleration.z = 9.81 + math.sin(t * 3) * 0.05
        
        # Set covariance (optional, all zeros is also fine)
        msg.orientation_covariance = [0.0] * 9
        msg.angular_velocity_covariance = [0.0] * 9
        msg.linear_acceleration_covariance = [0.0] * 9
        
        return msg

    def __create_fake_odom_msg(self):
        """Create a fake odometry message with zero values"""
        msg = Odometry()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "odom"
        msg.child_frame_id = "base_link"
        
        # All zero position
        msg.pose.pose.position.x = 0.0
        msg.pose.pose.position.y = 0.0
        msg.pose.pose.position.z = 0.0
        
        # Identity quaternion
        msg.pose.pose.orientation.x = 0.0
        msg.pose.pose.orientation.y = 0.0
        msg.pose.pose.orientation.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        
        # Zero velocity
        msg.twist.twist.linear.x = 0.0
        msg.twist.twist.linear.y = 0.0
        msg.twist.twist.linear.z = 0.0
        msg.twist.twist.angular.x = 0.0
        msg.twist.twist.angular.y = 0.0
        msg.twist.twist.angular.z = 0.0
        
        # Zero covariance
        msg.pose.covariance = [0.0] * 36
        msg.twist.covariance = [0.0] * 36
        
        return msg

    def __imu_send_thread(self):
        """Send IMU data at 50 Hz"""
        rate = rospy.Rate(50)  # 50 Hz
        
        while not rospy.is_shutdown() and self.__running:
            if self.__has_connected:
                # Create fake IMU message
                imu_msg = self.__create_fake_imu_msg()
                
                # Serialize the message
                import io
                buff = io.BytesIO()
                imu_msg.serialize(buff)
                payload = buff.getvalue()
                
                # Send with IMU header: 0xab, 0xcd
                if self.__send_packet([0xab, 0xcd], payload):
                    rospy.logdebug("Sent IMU data")
                else:
                    rospy.logwarn_throttle(5, "Failed to send IMU data")
            
            rate.sleep()

    def __odom_send_thread(self):
        """Send odometry data at 20 Hz"""
        rate = rospy.Rate(20)  # 20 Hz
        
        while not rospy.is_shutdown() and self.__running:
            if self.__has_connected:
                # Create fake odometry message
                odom_msg = self.__create_fake_odom_msg()
                
                # Serialize the message
                import io
                buff = io.BytesIO()
                odom_msg.serialize(buff)
                payload = buff.getvalue()
                
                # Send with Odometry header: 0xab, 0xce
                if self.__send_packet([0xab, 0xce], payload):
                    rospy.logdebug("Sent Odometry data")
                else:
                    rospy.logwarn_throttle(5, "Failed to send Odometry data")
            
            rate.sleep()


if __name__ == "__main__":
    address = "127.0.0.1"
    port = 8091

    if len(sys.argv) > 1:
        address = sys.argv[1]

    if len(sys.argv) > 2:
        port = int(sys.argv[2])

    rospy.loginfo(f"Connecting to {address}:{port}")
    
    simulator = ImuOdomSimulator(address, port)
    try:
        simulator.run()
    except rospy.ROSInterruptException:
        pass
