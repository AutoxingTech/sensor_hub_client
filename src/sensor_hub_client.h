#pragma once
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <rplidar_ros/AxLaserScan.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>

#include "cln_msgs/HardwareCtrl.h"
#include "cln_msgs/SensorType.h"
#include "cln_msgs/CleanDeviceType.h"
#include "cln_msgs/HardwareState.h"
#include "cln_msgs/HardwareStateType.h"

#include "tcp_stream.h"
#include "tcp_pack.h"

class SensorHubClient : public ParserManagerDelegate
{
public:
    SensorHubClient();
    ~SensorHubClient() {}
    int run();

private: // callback
    void _imuCB(const sensor_msgs::Imu &msg);
    void _odomCB(const nav_msgs::Odometry &msg);
    void _laserCB(const rplidar_ros::AxLaserScan &msg);
    void _hwStateCB(const cln_msgs::HardwareState &msg);

    virtual void ParserManager_packetFound(const std::vector<uint8_t> &header, ros::Time time, const uint8_t *pack,
                                           size_t bytes) override;

private:
    ros::NodeHandle m_nh;

    // sub
    ros::Subscriber m_imuSub;
    ros::Subscriber m_odomSub;
    ros::Subscriber m_laserSub;
    ros::Subscriber m_hwStateSub;

    // pub
    ros::Publisher m_cmdVelPub;
    ros::Publisher m_modeControlPub;

    tf2_ros::TransformBroadcaster tf2_broadcaster;

    std::shared_ptr<TcpStream> m_comStream;

    // param
    std::string m_hostIP;
    int m_hostPort;

    // tcp parser
    ParserManager m_parserManager{this};
    MsgPackParser m_modeControlParser;
    MsgPackParser m_cmdVelParser;
    MsgPackParser m_tfParser;

    // tcp send buffer
    uint8_t m_buffer[1024];
};
