/**
 * @file function_set_floor.hpp
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __FUNCTION_SET_FLOOR_HPP__
#define __FUNCTION_SET_FLOOR_HPP__

 
#include <std_msgs/Int32.h>
#include "../../rdms_topic_factory.h"
#include "../../rdms_topic_base.h"
#include <string>
struct FunctionSetFloorReqData{
    std::string floor;
};

DEFINE_STRUCT_SCHEMA(FunctionSetFloorReqData,\
                    DEFINE_STRUCT_FIELD(floor, "floor"));
DEFINE_STRUCT_REQUEST_FIELD(FunctionSetFloorReqData);

namespace zros{

class FunctionSetFloor final: public RdmsTopicBase{
public:
    FunctionSetFloor(): ros_nh_(), ros_param_("~"){

    }

    void init() override{
        RdmsTopicBase::topicInit("function", "set-floor");
        subscribeTopic(getReqTopicName(), 
                        &FunctionSetFloor::setFloorCallback, this);
    }

private:
    ros::NodeHandle ros_nh_; //ros node handle
    ros::NodeHandle ros_param_;
    MqttRequestPayload<FunctionSetFloorReqData> topic_resquest_; 
    MqttResponePayload<emptyData> topic_respone_;

    void setFloorCallback(const std::string& msg){
        parseMessage(msg, topic_resquest_);
        std::string correlationId = topic_resquest_.correlationId;
        std::string clientId = topic_resquest_.clientId;
        int16_t timestamp = topic_resquest_.timestamp;
        std::string floor = topic_resquest_.data.floor;
        
        ros_param_.setParam("/robot_location/floor", atoi(floor.c_str()));

        topic_respone_.correlationId = correlationId;
        topic_respone_.clientId = getRobotID();
        topic_respone_.timestamp = getUnixTimeStamp();
        topic_respone_.result = "SUCCESS";
        topic_respone_.message = "I know that Iam on the " + floor + "floor";
        topic_respone_.data = {};
        publishTopic(getResTopicName(), topic_respone_);
    } 

}; //end class FunctionSetFloor

REGIST_MQTT_TOPIC(FunctionSetFloor, FunctionSetFloor);

}

#endif /*__FUNCTION_SET_FLOOR_HPP__*/