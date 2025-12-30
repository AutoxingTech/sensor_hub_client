#include <ros/ros.h>
#include "sensor_hub_client.h"

int main(int argc, char** argv)
{
    uint64_t v = 0x3f5a097280000000;
    double d = reinterpret_cast<double&>(v);
    printf("%lf\n", d); // 0.001589

    uint64_t u = reinterpret_cast<uint64_t&>(d);
    printf("0x%lx\n", u); // 0x3f5a097280000000

    ros::init(argc, argv, "sensor_hub_client");

    const std::string uri = ros::master::getURI();
    printf("master uri is %s\n", uri.c_str());
    ROS_INFO("master url is %s", uri.c_str());

    // 创建TPC client 需要做自动重连

    SensorHubClient client;

    return client.run();
}