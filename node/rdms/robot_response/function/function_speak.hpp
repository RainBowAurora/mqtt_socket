/**
 * @file function_speak.hpp
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __FUNCTION_SPEAK_HPP__
#define __FUNCTION_SPEAK_HPP__

 
#include <std_msgs/String.h>

#include "../../rdms_topic_factory.h"
#include "../../rdms_topic_base.h"

struct FunctionSpeakReqData{
    std::string content;
};

DEFINE_STRUCT_SCHEMA(FunctionSpeakReqData,\
                    DEFINE_STRUCT_FIELD(content, "content"));
DEFINE_STRUCT_REQUEST_FIELD(FunctionSpeakReqData);

namespace zros{

class FunctionSpeak final: public RdmsTopicBase{
public:
    FunctionSpeak(): ros_nh_(){

    }

    void init() override{
        RdmsTopicBase::topicInit("function", "speak");
        subscribeTopic(getReqTopicName(), 
                        &FunctionSpeak::speakCallback, this);

        pub_speak_ = ros_nh_.advertise<std_msgs::String>("/tts_play", 10, false);
    }

private:
    ros::NodeHandle ros_nh_; //ros node handle
    ros::Publisher pub_speak_;
    MqttRequestPayload<FunctionSpeakReqData> topic_resquest_; 
    MqttResponePayload<emptyData> topic_respone_;

    void speakCallback(const std::string& msg){
        parseMessage(msg, topic_resquest_);
        std::string correlationId = topic_resquest_.correlationId;
        std::string clientId = topic_resquest_.clientId;
        int16_t timestamp = topic_resquest_.timestamp;
        std::string content = topic_resquest_.data.content;

        std_msgs::String temp_speak;
        temp_speak.data = content;
        pub_speak_.publish(temp_speak);


        topic_respone_.correlationId = correlationId;
        topic_respone_.clientId = getRobotID();
        topic_respone_.timestamp = getUnixTimeStamp();
        topic_respone_.result = "SUCCESS";
        topic_respone_.message = "I am gooint to speak: (" + content + ")";
        publishTopic(getResTopicName(), topic_respone_);
    }

}; //end class FunctionSpeak

REGIST_MQTT_TOPIC(FunctionSpeak, FunctionSpeak);

} //end namespace zros

#endif /*__FUNCTION_SPEAK_HPP__*/