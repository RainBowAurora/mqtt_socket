/**
 * @file function_control_door.hpp
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __FUNCTION_CONTROL_DOOR_HPP__
#define __FUNCTION_CONTROL_DOOR_HPP__

 
#include <std_msgs/Int32.h>
#include "../../rdms_topic_factory.h"
#include "../../rdms_topic_base.h"

struct FunctionControlDoorReqData{
    int doorNumber;
    std::string command;
}; //end struct FunctionControlDoorReqData
DEFINE_STRUCT_SCHEMA(FunctionControlDoorReqData,\
                    DEFINE_STRUCT_FIELD(doorNumber, "doorNumber"),\
                    DEFINE_STRUCT_FIELD(command, "command"));
DEFINE_STRUCT_REQUEST_FIELD(FunctionControlDoorReqData);

namespace zros{

class FunctionControlDoor final: public RdmsTopicBase{
public:
    FunctionControlDoor(): ros_nh_(){

    }

    void init() override{
        RdmsTopicBase::topicInit("function", "control-door");
        subscribeTopic(getReqTopicName(), 
                        &FunctionControlDoor::controlDoorCallback, this);

        sub_unlock_ = ros_nh_.advertise<std_msgs::Int32>("/unlock", 10);
    }

private:
    ros::NodeHandle ros_nh_; //ros node handle
    ros::Publisher sub_unlock_;
    MqttRequestPayload<FunctionControlDoorReqData> topic_resquest_; 
    MqttResponePayload<emptyData> topic_respone_; 

    void controlDoorCallback(const std::string& msg){
        parseMessage(msg, topic_resquest_);
        std::string correlationId = topic_resquest_.correlationId;
        int doorNumber = topic_resquest_.data.doorNumber;
        std::string command = topic_resquest_.data.command;

        topic_respone_.correlationId = topic_resquest_.correlationId;
        topic_respone_.clientId = getRobotID();
        topic_respone_.timestamp = getUnixTimeStamp();
        if(command == "OPEN"){
            std_msgs::Int32 temp_unlock;
            temp_unlock.data = topic_resquest_.data.doorNumber;
            sub_unlock_.publish(temp_unlock);
            topic_respone_.result = "SUCCESS";
            topic_respone_.message = "Open the door" + std::to_string(doorNumber);
        }else{
            topic_respone_.result = "SUCCESS";
            topic_respone_.message = "Unable to close the door" + std::to_string(doorNumber); 
        }
        
        publishTopic(getResTopicName(), topic_respone_);
    } 

}; //end class FunctionControlDoor

REGIST_MQTT_TOPIC(FunctionControlDoor, FunctionControlDoor);

} //end namespace zros

#endif /*__FUNCTION_CONTROL_DOOR_HPP__*/