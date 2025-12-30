#include <ros/ros.h>
#include "sensor_hub_client.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_hub_client");

    const std::string uri = ros::master::getURI();
    printf("master uri is %s\n", uri.c_str());
    ROS_INFO("master url is %s", uri.c_str());

    // 创建TCP client 需要做自动重连

    SensorHubClient client;

    return client.run();
}