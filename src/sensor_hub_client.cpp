#include "sensor_hub_client.h"
#include "sensor_hub_client/TcpRobotControl.h"
#include "sensor_hub_client/TcpRobotState.h"
#include "crc.h"

SensorHubClient::SensorHubClient()
    : m_nh("~"), m_imuParser({0xab, 0xcd}), m_asyncSpinner(1, &m_callbackQueue), m_asyncHandle("~")
{
    m_asyncHandle.setCallbackQueue(&m_callbackQueue);
    m_asyncSpinner.start();

    m_parserManager.addParser(&m_imuParser);

    m_tcpStream = std::make_shared<TcpStream>();
}

int SensorHubClient::run()
{
    m_controlModeClient = m_nh.serviceClient<ax_msgs::SetControlMode>("/wheel_control/set_control_mode");
    m_lonYuImuPub = m_nh.advertise<sensor_msgs::Imu>("/lon_yu_imu", 100);

    m_nh.param<std::string>("host_ip", m_hostIP, "127.0.0.1");
    m_nh.param<int>("host_port", m_hostPort, 8091);

    setUserControlMode(UserControlMode::manual);

    uint8_t buffer[1024];
    ros::Rate rate(200);

    while (ros::ok())
    {
        // check connect is ok.
        if (!m_tcpStream->isConnected())
        {
            ROS_WARN("TCP client is disconnected, will reconnect...");
            m_tcpStream->close();
            if (!m_tcpStream->open(m_hostIP, m_hostPort))
            {
                // open fail.
                ros::Duration(1).sleep();
            }
            else
            {
                ROS_INFO("success connected %s:%d", m_hostIP.c_str(), m_hostPort);
            }
        }

        int sizeOut = m_tcpStream->read(buffer, sizeof(buffer));
        if (sizeOut > 0)
            m_parserManager.feed(buffer, sizeOut);
        ROS_DEBUG_THROTTLE(10, "read buffer size %d", sizeOut);
        rate.sleep();
    }

    return 0;
}

void SensorHubClient::ParserManager_packetFound(const std::vector<uint8_t>& header, ros::Time time,
                                                const uint8_t* packRaw, size_t bytes)
{
    MsgPack* pack = (MsgPack*)packRaw;
    if (header == m_imuParser.header())
    {
        sensor_msgs::Imu imu_msg;
        ros::serialization::IStream istream((uint8_t*)(pack->payload), pack->length);
        ros::serialization::deserialize(istream, imu_msg);

        m_lonYuImuPub.publish(imu_msg);
    }
}

void SensorHubClient::setUserControlMode(UserControlMode mode)
{
    ax_msgs::SetControlMode srv;
    srv.request.control_mode = UserControlMode_toString(mode);
    if (m_controlModeClient.call(srv))
    {
        if (srv.response.success)
        {
            ROS_INFO("Set user control mode to %s success.", srv.request.control_mode.c_str());
        }
        else
        {
            ROS_WARN("Set user control mode to %s failed.", srv.request.control_mode.c_str());
        }
    }
    else
    {
        ROS_ERROR("Failed to call service /wheel_control/set_control_mode");
    }
}
