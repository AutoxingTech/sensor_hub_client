配套的服务端地址: https://git.autoxing.com/liudf/sensor_hub_server


## 启动方法参考：

```
source /home/aoting/rplidar_ws/install/setup.bash
source /home/aoting/sensor_hub_client/install/setup.bash
rosrun sensor_hub_client client.py

```

## topic约定

rospy.Subscriber("/imu", Imu, self.__imu_callback)
rospy.Subscriber("/odom_origin", Odometry, self.__odometry_callback)
rospy.Subscriber("/hardware_state", HardwareState, self.__hardware_state_callback)
rospy.Subscriber("/ax_laser_scan", AxLaserScan, self.__laser_scan_callback)

publisher = rospy.Publisher("/cmd_vel", Twist, queue_size=20)
publisher = rospy.Publisher("/automode_ctrl", HardwareCtrl, queue_size=10)
