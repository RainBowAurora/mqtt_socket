#include <ros/ros.h>
#include <iostream>
#include "mqtt_socket.h"

using namespace zros;

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "mqtt_socket");

    MqttSocket mqtt_socket;
    mqtt_socket.run();

    return 0;
}