/**
 * @file function_reboot.hpp
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __FUNCTION_REBOOT_HPP__
#define __FUNCTION_REBOOT_HPP__

 
#include <std_msgs/String.h>
#include "../../rdms_topic_factory.h"
#include "../../rdms_topic_base.h"
namespace zros{

class FunctionReboot final: public RdmsTopicBase{
public:
    FunctionReboot(): ros_nh_(){

    }

    void init() override{
        RdmsTopicBase::topicInit("function", "reboot");
        subscribeTopic(getReqTopicName(), 
                        &FunctionReboot::rebootCallback, this);
        pub_speak_ = ros_nh_.advertise<std_msgs::String>("/sound_player", 10);       
    } 

private:
    ros::NodeHandle ros_nh_; //ros node handle
    ros::Publisher pub_speak_;
    MqttRequestPayload<emptyData> topic_resquest_; 
    MqttResponePayload<emptyData> topic_respone_;

    void rebootCallback(const std::string& msg){
        parseMessage(msg, topic_resquest_);
        std::string correlationId = topic_resquest_.correlationId;
        std::string clientId = topic_resquest_.clientId;
        int16_t timestamp = topic_resquest_.timestamp;

        std_msgs::String temp_speak;
        temp_speak.data = "reboot";
        pub_speak_.publish(temp_speak);

        topic_respone_.correlationId = correlationId;
        topic_respone_.clientId = getRobotID();
        topic_respone_.timestamp = getUnixTimeStamp();
        topic_respone_.result = "SUCCESS";
        topic_respone_.message = "I am going to reboot";

        publishTopic(getResTopicName(), topic_respone_);
        //system reboot cmd is here
        std::cout << SystemWithResult("echo zhen123 | sudo -S reboot");

    }
}; //end class FunctionReboot

REGIST_MQTT_TOPIC(FunctionReboot, FunctionReboot);

} //end namespace zros

#endif /*__FUNCTION_REBOOT_HPP__*/