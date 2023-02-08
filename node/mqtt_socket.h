/**
 * @file mqtt_socket.h
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __RDMS_MQTT_SOCKET_H__
#define __RDMS_MQTT_SOCKET_H__

#include <ros/ros.h>
#include <iostream>
#include <memory>

#include "commander/robot_requirest/requirest.h"
#include "commander/robot_info/info.h"

#include "rdms/robot_info/info.h"
#include "rdms/robot_response/response.h"
#include "rdms/robot_requirest/requirest.h"

namespace zros{
class MqttSocket final{
public:
    MqttSocket();
    ~MqttSocket();
    void run();
private:
    ros::NodeHandle ros_nh_;
    ros::NodeHandle ros_param_;
    ros::Timer timer_;
    void rdmsInit();
    void comderInit();
    void uploadParam(const ros::TimerEvent& e);
    std::vector<std::shared_ptr<RdmsTopicBase>> mqtt_rdms_vec_; 
    std::vector<std::shared_ptr<ComderTopicBase>> mqtt_comder_vec_;
    MqttSocket(const MqttSocket&) = delete;
    MqttSocket(MqttSocket&&) = delete;
    MqttSocket& operator=(const MqttSocket&) = delete;
    MqttSocket& operator=(MqttSocket&&) = delete;
    
}; //end class MqttSocket

} //end namespace zros


#endif /*__RDMS_MQTT_SOCKET_H__*/