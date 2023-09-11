#include "sensor_hub_client.h"
#include "sensor_hub_client/TcpRobotControl.h"
#include "crc.h"

using namespace cln_msgs;

SensorHubClient::SensorHubClient() : m_nh("~"), m_modeControlParser({0xba, 0xe1}),
                                     m_cmdVelParser({0xba, 0xe2}), m_tfParser({0xba, 0xe3}),
                                     m_asyncSpinner(1, &m_callbackQueue), m_asyncHandle("~")
{
    m_asyncHandle.setCallbackQueue(&m_callbackQueue);
    m_asyncSpinner.start();

    m_parserManager.addParser(&m_cmdVelParser);
    m_parserManager.addParser(&m_modeControlParser);
    m_parserManager.addParser(&m_tfParser);

    m_comStream = std::make_shared<TcpStream>();
}

int SensorHubClient::run()
{
    m_imuSub = m_asyncHandle.subscribe("/imu", 100, &SensorHubClient::_imuCB, this);
    m_odomSub = m_asyncHandle.subscribe("/odom_origin", 50, &SensorHubClient::_odomCB, this);
    m_laserSub = m_asyncHandle.subscribe("/ax_laser_scan", 10, &SensorHubClient::_laserCB, this);
    m_hwStateSub = m_asyncHandle.subscribe("/hardware_state", 20, &SensorHubClient::_hwStateCB, this);

    m_cmdVelPub = m_nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    m_modeControlPub = m_nh.advertise<cln_msgs::HardwareCtrl>("/automode_ctrl", 10);

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

        int sizeOut = m_comStream->read(buffer, sizeof(buffer));
        if (sizeOut > 0)
            m_parserManager.feed(buffer, sizeOut);
        ROS_INFO_THROTTLE(10, "read buffer size %d", sizeOut);
    }

    return 0;
}

// send socket.
void SensorHubClient::_imuCB(const sensor_msgs::Imu &msg)
{
    ROS_INFO_THROTTLE(10, "_imuCB");

    uint32_t payloadLength = ros::serialization::serializationLength(msg);
    MsgPack *pack = (MsgPack *)alloca(sizeof(MsgPack) + payloadLength);
    ros::serialization::OStream stream(pack->payload, payloadLength);
    ros::serialization::serialize(stream, msg);

    pack->header[0] = 0xab;
    pack->header[1] = 0xcd;
    pack->length = payloadLength;
    pack->crc = calculateCRC16(pack->payload, pack->length);

    int sizeOut = m_comStream->write((uint8_t *)pack, sizeof(MsgPack) + payloadLength);
    ROS_INFO_THROTTLE(10, "write length is %d", sizeOut);
}

void SensorHubClient::_odomCB(const nav_msgs::Odometry &msg)
{
    ROS_INFO_THROTTLE(10, "_odomCB");

    uint32_t payloadLength = ros::serialization::serializationLength(msg);
    MsgPack *pack = (MsgPack *)alloca(sizeof(MsgPack) + payloadLength);
    ros::serialization::OStream stream(pack->payload, payloadLength);
    ros::serialization::serialize(stream, msg);

    pack->header[0] = 0xab;
    pack->header[1] = 0xce;
    pack->length = payloadLength;
    pack->crc = calculateCRC16(pack->payload, pack->length);

    int sizeOut = m_comStream->write((uint8_t *)pack, sizeof(MsgPack) + payloadLength);
    ROS_INFO_THROTTLE(10, "write length is %d", sizeOut);
}

void SensorHubClient::_laserCB(const rplidar_ros::AxLaserScan &msg)
{
    ROS_INFO_THROTTLE(10, "_laserCB");

    uint32_t payloadLength = ros::serialization::serializationLength(msg);
    MsgPack *pack = (MsgPack *)alloca(sizeof(MsgPack) + payloadLength);
    ros::serialization::OStream stream(pack->payload, payloadLength);
    ros::serialization::serialize(stream, msg);

    pack->header[0] = 0xab;
    pack->header[1] = 0xcf;
    pack->length = payloadLength;
    pack->crc = calculateCRC16(pack->payload, pack->length);

    int sizeOut = m_comStream->write((uint8_t *)pack, sizeof(MsgPack) + payloadLength);
    ROS_INFO_THROTTLE(10, "write length is %d", sizeOut);
}

void SensorHubClient::_hwStateCB(const cln_msgs::HardwareState &msg)
{
    ROS_INFO_THROTTLE(10, "_hwStateCB");

    uint32_t payloadLength = ros::serialization::serializationLength(msg);
    MsgPack *pack = (MsgPack *)alloca(sizeof(MsgPack) + payloadLength);
    ros::serialization::OStream stream(pack->payload, payloadLength);
    ros::serialization::serialize(stream, msg);

    pack->header[0] = 0xab;
    pack->header[1] = 0xd0;
    pack->length = payloadLength;
    pack->crc = calculateCRC16(pack->payload, pack->length);

    int sizeOut = m_comStream->write((uint8_t *)pack, sizeof(MsgPack) + payloadLength);
    ROS_INFO_THROTTLE(10, "write length is %d", sizeOut);
}

void SensorHubClient::ParserManager_packetFound(const std::vector<uint8_t> &header, ros::Time time, const uint8_t *packRaw,
                                                size_t bytes)
{
    MsgPack *pack = (MsgPack *)packRaw;
    if (header == m_cmdVelParser.header())
    {
        // deserialize
        static geometry_msgs::Twist twist;
        ros::serialization::IStream istream((uint8_t *)(pack->payload), pack->length);
        ros::serialization::deserialize(istream, twist);
        m_cmdVelPub.publish(twist);
    }
    else if (header == m_modeControlParser.header())
    {
        // deserialize
        static sensor_hub_client::TcpRobotControl robotControl;
        ros::serialization::IStream istream((uint8_t *)(pack->payload), pack->length);
        ros::serialization::deserialize(istream, robotControl);

        ROS_INFO_STREAM("robotControl.control_mode is" << robotControl.enable_wheels);

        HardwareCtrl ctrl;
        ctrl.sensor_id.push_back(SensorType());
        ctrl.device_id.push_back(CleanDeviceType());
        if (robotControl.enable_wheels)
        {
            HardwareStateType stateType;
            stateType.state = HardwareStateType::ON;
            ctrl.state.push_back(stateType); // auto
        }
        else
        {
            HardwareStateType stateType;
            stateType.state = HardwareStateType::OFF;
            ctrl.state.push_back(stateType); // manual
        }
    }
    else if (header == m_tfParser.header())
    {
        // deserialize
        tf2_msgs::TFMessage tf_msg;
        ros::serialization::IStream istream((uint8_t *)(pack->payload), pack->length);
        ros::serialization::deserialize(istream, tf_msg);

        for (const auto &transform : tf_msg.transforms)
        {
            m_tf2Broadcaster.sendTransform(transform);
        }
    }
}
