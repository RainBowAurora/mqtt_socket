/**
 * @file function_set_driving_mode.hpp
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __FUNCTION_SET_DRIVING_MODE_HPP__
#define __FUNCTION_SET_DRIVING_MODE_HPP__

 
#include <std_msgs/Int32.h>
#include "../../rdms_topic_factory.h"
#include "../../rdms_topic_base.h"

struct FunctionSetDrivingModeReqData{
    std::string drivingMode;
};
DEFINE_STRUCT_SCHEMA(FunctionSetDrivingModeReqData,\
                    DEFINE_STRUCT_FIELD(drivingMode, "drivingMode"));
DEFINE_STRUCT_REQUEST_FIELD(FunctionSetDrivingModeReqData);

namespace zros{

class FunctionSetDrivingMode final: public RdmsTopicBase{
public:
    FunctionSetDrivingMode(): ros_nh_(){

    }

    void init() override{
        RdmsTopicBase::topicInit("function", "set-driving-mode");
        subscribeTopic(getReqTopicName(), 
                        &FunctionSetDrivingMode::setDrivingModeCallback, this);
        pub_start_flag_ = ros_nh_.advertise<std_msgs::Int32>("/start_flag", 1, true);
    }

private:
    ros::NodeHandle ros_nh_; //ros node handle
    ros::Publisher pub_start_flag_;
    MqttRequestPayload<FunctionSetDrivingModeReqData> topic_resquest_; 
    MqttResponePayload<emptyData> topic_respone_;

    void setDrivingModeCallback(const std::string& msg){
        parseMessage(msg, topic_resquest_);
        std::string correlationId = topic_resquest_.correlationId;
        std::string clientId = topic_resquest_.clientId;
        int16_t timestamp = topic_resquest_.timestamp;
        std::string drivingMode = topic_resquest_.data.drivingMode;

        std_msgs::Int32 temp_start_flag;
        temp_start_flag.data = (drivingMode == "AUTO"? 1 : 0);
        pub_start_flag_.publish(temp_start_flag);

        topic_respone_.correlationId = correlationId;
        topic_respone_.clientId = getRobotID();
        topic_respone_.timestamp = getUnixTimeStamp();
        topic_respone_.result = "SUCCESS";
        topic_respone_.message = "I am in the " + drivingMode + "driving mode";

        publishTopic(getResTopicName(), topic_respone_);
    } 

}; //end class FunctionSetDrivingMode

REGIST_MQTT_TOPIC(FunctionSetDrivingMode, FunctionSetDrivingMode);

} //end namespace zros

#endif /*__FUNCTION_SET_DRIVING_MODE_HPP__*/