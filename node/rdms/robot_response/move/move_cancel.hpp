/**
 * @file move_cancel.hpp
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __MOVE_CANCEL_HPP__
#define __MOVE_CANCEL_HPP__   

 
#include "../../rdms_topic_factory.h"
#include "../../rdms_topic_base.h"
#include <std_msgs/String.h>

struct MoveCancelReqData{
    int moveId;
};
DEFINE_STRUCT_SCHEMA(MoveCancelReqData,\
                    DEFINE_STRUCT_FIELD(moveId, "moveId"));
DEFINE_STRUCT_REQUEST_FIELD(MoveCancelReqData);

namespace zros{

class MoveCancel final: public RdmsTopicBase{
public:
    MoveCancel(): ros_nh_(){

    }

    void init() override{
        RdmsTopicBase::topicInit("move", "cancel");
        subscribeTopic(getReqTopicName(), 
                        &MoveCancel::cancelCallback, this);

        pub_move_status_ = ros_nh_.advertise<std_msgs::String>("/task_destination", 1);
        pub_mission_cancel_ = ros_nh_.advertise<std_msgs::String>("/mission_cancel", 1);
    }

private:
    ros::NodeHandle ros_nh_; //ros node handle
    ros::Publisher pub_mission_cancel_;
    ros::Publisher pub_move_status_;
    MqttRequestPayload<MoveCancelReqData> topic_resquest_; 
    MqttResponePayload<emptyData> topic_respone_;

    void cancelCallback(const std::string& msg){
        parseMessage(msg, topic_resquest_);
        std::string correlationId = topic_resquest_.correlationId;
        int moveId = topic_resquest_.data.moveId;

        topic_respone_.correlationId = correlationId;
        topic_respone_.clientId = getRobotID();
        topic_respone_.timestamp = getUnixTimeStamp();

        std_msgs::String mission_cancel;
        mission_cancel.data = std::to_string(moveId);
        pub_mission_cancel_.publish(mission_cancel);

        std_msgs::String temp_move_status;
        temp_move_status.data = "cancel";
        pub_move_status_.publish(temp_move_status);

        topic_respone_.result = "SUCCESS";
        topic_respone_.message = "canceled moving to " + std::to_string(moveId);
        topic_respone_.data = {};
        publishTopic(getResTopicName(), topic_respone_);
    }

}; //end class MoveCancel

REGIST_MQTT_TOPIC(MoveCancel, MoveCancel);

} //end namespace zros


#endif /*__MOVE_CANCEL_HPP__*/

