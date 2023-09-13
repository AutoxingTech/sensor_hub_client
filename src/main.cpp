#include <ros/ros.h>
#include "sensor_hub_client.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sensor_hub_client");

    // 创建TPC client 需要做自动重连

    SensorHubClient client;

    return client.run();
}