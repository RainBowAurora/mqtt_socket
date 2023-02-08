/**
 * @file move_dispatch.hpp
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __MOVE_DISPATCH_HPP__
#define __MOVE_DISPATCH_HPP__

 
#include "../../rdms_topic_factory.h"
#include "../../rdms_topic_base.h"
#include <zr_msgs/mission.h>
#include <zr_msgs/task.h>
#include <zr_msgs/order_info.h>

struct MoveDispatchReqData{
    int moveId;
    std::string destination;
};

DEFINE_STRUCT_SCHEMA(MoveDispatchReqData,\
                    DEFINE_STRUCT_FIELD(moveId, "moveId"),\
                    DEFINE_STRUCT_FIELD(destination, "destination"));
DEFINE_STRUCT_REQUEST_FIELD(MoveDispatchReqData);

namespace zros{

class MoveDispatch final: public RdmsTopicBase{
public:
    MoveDispatch(): ros_nh_(){

    }

    void init() override{
        RdmsTopicBase::topicInit("move", "dispatch");
        subscribeTopic(getReqTopicName(), 
                        &MoveDispatch::moveDispatchCallback, this);
        pub_move_status_ = ros_nh_.advertise<std_msgs::String>("/task_destination", 1);
        pub_move_mission_ = ros_nh_.advertise<zr_msgs::mission>("/mission", 1);
    }
    
private:
    ros::NodeHandle ros_nh_; //ros node handle  
    ros::Publisher pub_move_status_;
    ros::Publisher pub_move_mission_;
    MqttRequestPayload<MoveDispatchReqData> topic_resquest_; 
    MqttResponePayload<emptyData> topic_respone_;

    void moveDispatchCallback(const std::string& msg){
        parseMessage(msg, topic_resquest_);
        std::string correlationId = topic_resquest_.correlationId;
        int moveId = topic_resquest_.data.moveId;
        std::string destination = topic_resquest_.data.destination;

        std_msgs::String temp_move_status;
        temp_move_status.data = destination;
        pub_move_status_.publish(temp_move_status);

        zr_msgs::mission temp_mission;
        zr_msgs::task temp_move_task;
        temp_mission.mission_id = moveId;
        temp_move_task.task_type = "MOVE";
        temp_move_task.task_id = std::to_string(moveId);
        temp_move_task.dest_id = destination;
        temp_move_task.priority = 0;
        temp_mission.task_array.push_back(temp_move_task);

        pub_move_mission_.publish(temp_mission);

        topic_respone_.correlationId = correlationId;
        topic_respone_.clientId = getRobotID();
        topic_respone_.result = "SUCCESS";
        topic_respone_.message = "on manual mode, I am ready to go to " + destination;
        topic_respone_.data = {};

        publishTopic(getResTopicName(), topic_respone_);
    }

}; //end namespace MoveDispatch

REGIST_MQTT_TOPIC(MoveDispatch, MoveDispatch);

} //end namespace zros


#endif /*__MOVE_DISPATCH_HPP__*/