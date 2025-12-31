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

    m_wheelStateSub =
        m_asyncHandle.subscribe("/wheel_control/wheel_state", 20, &SensorHubClient::wheelStateCallback, this);

    m_nh.param<std::string>("host_ip", m_hostIP, "127.0.0.1");
    m_nh.param<int>("host_port", m_hostPort, 8091);

    setUserControlMode(m_userControlModeStr);

    // 创建1秒一次的定时器
    m_commonTimer = m_nh.createSteadyTimer(ros::WallDuration(0.5), &SensorHubClient::commonTimerCallback, this);

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

void SensorHubClient::setUserControlMode(const std::string& mode)
{
    ax_msgs::SetControlMode srv;
    srv.request.control_mode = mode;
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

void SensorHubClient::wheelStateCallback(const ax_msgs::WheelState::ConstPtr& msg)
{
    if (msg->control_mode != m_userControlModeStr)
    {
        m_userControlModeStr = msg->control_mode;
        m_userControlMode = UserControlMode_fromString(m_userControlModeStr.c_str());
    }
}

void SensorHubClient::requestWheelEnable()
{
    std_msgs::Bool msg;
    msg.data = (m_userControlMode == UserControlMode::automatic || m_userControlMode == UserControlMode::remote);

    uint32_t payloadLength = ros::serialization::serializationLength(msg);
    MsgPack* pack = (MsgPack*)alloca(sizeof(MsgPack) + payloadLength);
    ros::serialization::OStream stream(pack->payload, payloadLength);
    ros::serialization::serialize(stream, msg);
    pack->header[0] = 0xba;
    pack->header[1] = 0xe1;
    pack->length = payloadLength;
    pack->crc = calculateCRC16(pack->payload, pack->length);

    int sizeOut = m_tcpStream->write((uint8_t*)pack, sizeof(MsgPack) + payloadLength);
    ROS_DEBUG_THROTTLE(10, "write length is %d", sizeOut);
}

void SensorHubClient::commonTimerCallback(const ros::SteadyTimerEvent& event)
{
    requestWheelEnable();
}