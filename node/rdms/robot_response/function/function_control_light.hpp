/**
 * @file function_control_light.hpp
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __FUNCTION_CONTROL_LIGHT_HPP__
#define __FUNCTION_CONTROL_LIGHT_HPP__

 
#include <zr_msgs/LED.h>
#include <std_msgs/Int32.h>
#include "../../rdms_topic_factory.h"
#include "../../rdms_topic_base.h"

struct FunctionControlLightReqData{
    std::string lightType;
    std::string command;
}; //end struct FunctionControlLightReqData
DEFINE_STRUCT_SCHEMA(FunctionControlLightReqData,\
                    DEFINE_STRUCT_FIELD(lightType, "lightType"),\
                    DEFINE_STRUCT_FIELD(command, "command"));
DEFINE_STRUCT_REQUEST_FIELD(FunctionControlLightReqData);

namespace zros{

class FunctionControlLight final: public RdmsTopicBase{
public:
    FunctionControlLight(): ros_nh_(){

    }

    void init() override{
        RdmsTopicBase::topicInit("function", "control-light");
        subscribeTopic(getReqTopicName(), 
                        &FunctionControlLight::controlLightCallback, this);
        
        pub_front_light_ = ros_nh_.advertise<std_msgs::Int32>("/light_control",1, true);
        pub_rear_light_ = ros_nh_.advertise<zr_msgs::LED>("/led_control", 1, true);
        pub_flag_light_ = ros_nh_.advertise<std_msgs::Int32>("/flag_light", 1, true);
    } 

private:
    ros::NodeHandle ros_nh_; //ros node handle
    ros::Publisher pub_front_light_;
    ros::Publisher pub_rear_light_;
    ros::Publisher pub_flag_light_;
    MqttRequestPayload<FunctionControlLightReqData> topic_resquest_; 
    MqttResponePayload<emptyData> topic_respone_; 

    void controlLightCallback(const std::string& msg){
        parseMessage(msg, topic_resquest_);
        std::string correlationId = topic_resquest_.correlationId;
        std::string lightType = topic_resquest_.data.lightType;
        std::string command = topic_resquest_.data.command;

        topic_respone_.correlationId = correlationId;
        topic_respone_.clientId = getRobotID();
        topic_respone_.timestamp = getUnixTimeStamp();
        topic_respone_.result  = "SUCCESS"; 
        topic_respone_.message = "switched the " + lightType + command;

        if(lightType == "FRONT"){
            std_msgs::Int32 temp_front_light;
            temp_front_light.data = (command == "ON"? 1 : 0);
            pub_front_light_.publish(temp_front_light);
        }else if(lightType == "REAR"){
            zr_msgs::LED temp_rear_light;
            temp_rear_light.r = temp_rear_light.g = temp_rear_light.b = (command == "ON"? 255 : 0);
            temp_rear_light.mode = 0;
            temp_rear_light.t = 1;
            pub_rear_light_.publish(temp_rear_light);
        }else if(lightType == "FLAG"){
            std_msgs::Int32 temp_flag_light;
            temp_flag_light.data = (command == "ON"? 1: 0);
            pub_flag_light_.publish(temp_flag_light);
        }

        publishTopic(getResTopicName(), topic_respone_);
    
    }
}; //end class FunctionControlLight

REGIST_MQTT_TOPIC(FunctionControlLight, FunctionControlLight);


} //end namespace zros
#endif /*__FUNCTION_CONTROL_LIGHT_HPP__*/