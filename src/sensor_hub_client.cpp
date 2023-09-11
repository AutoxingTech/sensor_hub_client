#include "sensor_hub_client.h"

int SensorHubClient::run()
{
    m_imuSub = m_nh.subscribe("/imu", 100, &SensorHubClient::_imuCB, this);
    m_odomSub = m_nh.subscribe("/odom_origin", 50, &SensorHubClient::_odomCB, this);
    m_laserSub = m_nh.subscribe("/ax_laser_scan", 10, &SensorHubClient::_laserCB, this);
    m_hwStateSub = m_nh.subscribe("/hardware_state", 20, &SensorHubClient::_hwStateCB, this);

    m_cmdVelPub = m_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    m_modeControlPub = m_nh.advertise<cln_msgs::HardwareCtrl>("/automode_ctrl", 10);

    m_comStream = std::make_shared<TcpStream>();
    uint8_t buffer[1024];

    m_nh.param<std::string>("host_ip", m_hostIP, "0.0.0.0");
    m_nh.param<int>("host_port", m_hostPort, 8091);

    while (ros::ok())
    {
        // check connect is ok.
        if (!m_comStream->isConnected())
        {
            ROS_WARN("TCP client is disconnected, will reconnect...");
            m_comStream->close();
            if (!m_comStream->open(m_hostIP, m_hostPort))
            {
                // open fail.
                ros::Duration(1).sleep();
            }
            else
            {
                ROS_INFO("success connected %s:%d", m_hostIP.c_str(), m_hostPort);
            }
        }

        size_t readSize = m_comStream->read(buffer, 1024);
        // feed(buffer, readSize);
    }

    return 0;
}

// send socket.
void SensorHubClient::_imuCB(const sensor_msgs::Imu &msg)
{
}

void SensorHubClient::_odomCB(const nav_msgs::Odometry &msg)
{
}

void SensorHubClient::_laserCB(const rplidar_ros::AxLaserScan &msg)
{
}

void SensorHubClient::_hwStateCB(const cln_msgs::HardwareState &msg)
{
}