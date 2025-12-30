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


## 沐点项目配置：

```

wheel_control:
  device_type: tcp_sensor

tcp_sensor_hub:
  enable: true # 只有 wheel_control 走 tcp 传输逻辑，需要启动 sensor_hub_server
  enable_control_wheel: true # 沐点默认为 true
  send_tf: false # 程序中默认为 true，沐点不需要下发 tf
  odom: # 新版本的参数
    use_local_time: true
    send_message_type: "porting_odom"
```
