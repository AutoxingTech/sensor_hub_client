## 启动方法参考：

```
source devel/setup.bash

rosrun sensor_hub_client sensor_hub_client
# 或
roslaunch sensor_hub_client client.launch
```

## topic约定

```
m_imuSub = m_asyncHandle.subscribe("/imu", 100, &SensorHubClient::_imuCB, this);
m_odomSub = m_asyncHandle.subscribe("/odom_origin", 50, &SensorHubClient::_odomCB, this);
m_laserSub = m_asyncHandle.subscribe("/ax_laser_scan", 10, &SensorHubClient::_laserCB, this);
m_hwStateSub = m_asyncHandle.subscribe("/hardware_state", 20, &SensorHubClient::_hwStateCB, this);

m_cmdVelPub = m_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
m_modeControlPub = m_nh.advertise<cln_msgs::HardwareCtrl>("/automode_ctrl", 10);
```
