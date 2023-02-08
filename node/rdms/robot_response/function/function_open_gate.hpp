/**
 * @file function_open_gate.hpp
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __FUNCTION_OPEN_GATE_HPP__
#define __FUNCTION_OPEN_GATE_HPP__

 
#include "../../rdms_topic_factory.h"
#include "../../rdms_topic_base.h"
#include <std_msgs/Bool.h>

namespace zros{

class FunctionOpenGate final: public RdmsTopicBase{
public:
    FunctionOpenGate(): ros_nh_(){
        
    }

    void init() override{
        RdmsTopicBase::topicInit("function", "open-gate");
        subscribeTopic(getReqTopicName(), 
                        &FunctionOpenGate::openGateCallback, this);
        pub_automatic_door_ = ros_nh_.advertise<std_msgs::Bool>("/automatic_door/retryOpen", 1);

        sub_automatic_door_reault_ = ros_nh_.subscribe("/automatic_door/passResult", 1, 
                                            &FunctionOpenGate::rosAutoDoorResultCallback, this);
    } 

private:
    ros::NodeHandle ros_nh_; //ros node handle
    ros::Publisher pub_automatic_door_;
    ros::Subscriber sub_automatic_door_reault_;
    MqttRequestPayload<emptyData> topic_resquest_;
    MqttResponePayload<emptyData> topic_respone_;
    std_msgs::Bool auto_door_result_;
    void rosAutoDoorResultCallback(const std_msgs::Bool& msg){
        auto_door_result_ = msg;
    }

    void openGateCallback(const std::string& msg){
        parseMessage(msg, topic_resquest_);
        int64_t timestamp = topic_resquest_.timestamp;


        topic_respone_.correlationId = topic_resquest_.correlationId;
        topic_respone_.clientId = getRobotID();
        topic_respone_.timestamp = getUnixTimeStamp();

        std_msgs::Bool flag_open_automatic_door;
        flag_open_automatic_door.data = true;
        pub_automatic_door_.publish(flag_open_automatic_door);
        if(auto_door_result_.data){
            topic_respone_.result = "SUCCESS";
            topic_respone_.message = "I opened the gate.";
        }else{
            topic_respone_.result = "FAIL";
            topic_respone_.message = "I can't opened the gate."; 
        }
        topic_respone_.data = {};
        publishTopic(getResTopicName(), topic_respone_);
    }
}; //end class FunctionOpenGate

REGIST_MQTT_TOPIC(FunctionOpenGate, FunctionOpenGate);

} //end namespace zros

#endif /*__FUNCTION_OPEN_GATE_HPP__*/