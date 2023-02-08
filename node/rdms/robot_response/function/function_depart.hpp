/**
 * @file function_depart.hpp
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __FUNCTION_DEPART_HPP__
#define __FUNCTION_DEPART_HPP__

 
#include "../../rdms_topic_factory.h"
#include "../../rdms_topic_base.h"
#include <std_msgs/Bool.h>

struct FunctionDepartReqDeliveryId{
        int deliveryId;
};
DEFINE_STRUCT_SCHEMA(FunctionDepartReqDeliveryId,\
                    DEFINE_STRUCT_FIELD(deliveryId, "deliveryId"));
DEFINE_STRUCT_REQUEST_FIELD(FunctionDepartReqDeliveryId);

struct FunctionDepartReqMoveIdData{
    int moveId;
}; //end struct FunctionDepartReqData
DEFINE_STRUCT_SCHEMA(FunctionDepartReqMoveIdData,\
                    DEFINE_STRUCT_FIELD(moveId, "moveId"));
DEFINE_STRUCT_REQUEST_FIELD(FunctionDepartReqMoveIdData);

namespace zros{
    
class FunctionDepart final: public RdmsTopicBase{
public:
    FunctionDepart(): ros_nh_(){

    }

    void init() override{
        RdmsTopicBase::topicInit("function", "depart");
        subscribeTopic(getReqTopicName(), 
                        &FunctionDepart::departCallback, this);
        
        sub_move_destination_ = ros_nh_.subscribe("/task_destination", 1, 
                                    &FunctionDepart::rosMoveDestinationCallback, this);

        pub_mission_start_ = ros_nh_.advertise<std_msgs::Bool>("/mission_start", 1, true);
    } 

private:
    ros::NodeHandle ros_nh_; //ros node handle
    ros::Subscriber sub_move_destination_;
    ros::Publisher pub_mission_start_;
    std_msgs::String move_destination_;

    MqttRequestPayload<FunctionDepartReqDeliveryId> topic_delivery_request_; 
    MqttRequestPayload<FunctionDepartReqMoveIdData> topic_move_request_;
    MqttResponePayload<emptyData> topic_respone_;

    void rosMoveDestinationCallback(const std_msgs::String& msg){
        move_destination_ = msg;
    }

    void departCallback(const std::string& msg){
        auto result = nlohmann::json::parse(msg);
        if(result["data"].contains("deliveryId")){
            parseMessage(msg, topic_delivery_request_);
        }

        if(result["data"].contains("moveId")){
            parseMessage(msg, topic_move_request_);
        }
        std::string correlationId = topic_delivery_request_.correlationId;
        int deliveryId = topic_delivery_request_.data.deliveryId;
        int moveId = topic_move_request_.data.moveId;

        topic_respone_.correlationId = correlationId;
        topic_respone_.clientId = getRobotID();
        topic_respone_.timestamp = getUnixTimeStamp();
        topic_respone_.data = {};
        
        std_msgs::Bool mission_start;
        mission_start.data = true;
        pub_mission_start_.publish(mission_start);

        if(move_destination_.data == "cancel"){
            topic_respone_.result = "FAIL";
            topic_respone_.message = "destination not set";
        }else{
            topic_respone_.result = "SUCCESS";
            topic_respone_.message = "I am going to depart for " + move_destination_.data;
        }

        publishTopic(getResTopicName(), topic_respone_);
    }   
}; //end class FunctionDepart

REGIST_MQTT_TOPIC(FunctionDepart, FunctionDepart);

} //end namespace zros

#endif /*__FUNCTION_DEPART_HPP__*/