/**
 * @file function_ping_pong.hpp
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __FUNCTION_PING_PONG_HPP__
#define __FUNCTION_PING_PONG_HPP__

 
#include "../../rdms_topic_factory.h"
#include "../../rdms_topic_base.h"

namespace zros{

class FunctionPingPong final: public RdmsTopicBase{
public:
    FunctionPingPong(): ros_nh_(){

    }

    void init() override{
        RdmsTopicBase::topicInit("function", "ping-pong");
        subscribeTopic(getReqTopicName(), 
                        &FunctionPingPong::pingPongCallback, this);
    }    

private:
    ros::NodeHandle ros_nh_; //ros node handle
    MqttRequestPayload<emptyData> topic_resquest_; 
    MqttResponePayload<emptyData> topic_respone_;

    void pingPongCallback(const std::string& msg){
        parseMessage(msg, topic_resquest_);
        std::string correlationId = topic_resquest_.correlationId;
        std::string clientId = topic_resquest_.clientId;

        topic_respone_.correlationId = correlationId;
        topic_respone_.clientId  = getRobotID();
        topic_respone_.timestamp = getUnixTimeStamp();
        topic_respone_.result = "SUCCESS";
        topic_respone_.message = "receive ping from " + clientId;

        publishTopic(getResTopicName(), topic_respone_);
    }

}; //end class FunctionPingPong

REGIST_MQTT_TOPIC(FunctionPingPong, FunctionPingPong);

} //end namespace zros

#endif /*__FUNCTION_PING_PONG_HPP__*/