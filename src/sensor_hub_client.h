#pragma once
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <sensor_msgs/Imu.h>
#include <nav_msgs/Odometry.h>
#include <ax_msgs/SetControlMode.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_msgs/TFMessage.h>
#include <sensor_msgs/PointCloud2.h>

#include "tcp_stream.h"
#include "tcp_pack.h"
#include "common.h"

class SensorHubClient : public ParserManagerDelegate
{
public:
    SensorHubClient();
    ~SensorHubClient() {}
    int run();

private:
    // callback
    virtual void ParserManager_packetFound(const std::vector<uint8_t>& header, ros::Time time, const uint8_t* pack,
                                           size_t bytes) override;

    void setUserControlMode(UserControlMode mode);

private:
    ros::NodeHandle m_nh;
    ros::CallbackQueue m_callbackQueue;
    ros::AsyncSpinner m_asyncSpinner;
    ros::NodeHandle m_asyncHandle;

    // sub
    ros::Subscriber m_imuSub;

    // pub
    ros::Publisher m_cmdVelPub;

    tf2_ros::TransformBroadcaster m_tf2Broadcaster;

    std::shared_ptr<TcpStream> m_tcpStream;

    // param
    std::string m_hostIP;
    int m_hostPort;

    // tcp parser
    ParserManager m_parserManager{this};
    MsgPackParser m_modeControlParser;
    MsgPackParser m_cmdVelParser;
    MsgPackParser m_tfParser;

    ros::ServiceClient m_controlModeClient;
};
