#ifndef __COMDER_PASS_RESULT_H__
#define __COMDER_PASS_RESULT_H__

#include <std_msgs/Bool.h>
#include "../comder_topic_base.h"
#include "../comder_topic_factory.h"

struct PassResultinfoData{
    bool isSucceed;
    std::string doorId;
    std::string commandId;
}; 
DEFINE_STRUCT_SCHEMA(PassResultinfoData,\
    DEFINE_STRUCT_FIELD(isSucceed, "isSucceed"),\
    DEFINE_STRUCT_FIELD(doorId, "doorId"),\
    DEFINE_STRUCT_FIELD(commandId, "commandId"));
CMDER_DEFINE_STRUCT_INFO_FIELD(PassResultinfoData);

namespace zros{

class PassResult final: public ComderTopicBase{
public:
    PassResult(): ros_nh_(){

    }

    void init() override{
        ComderTopicBase::topicInit("pass-result");
        sub_pass_result_ = ros_nh_.subscribe("/automatic_door/passResult", 1,
                                &PassResult::rosPassResultCallback, this);
        sub_command_id_ = ros_nh_.subscribe("/automatic_door/permission", 1,
                                &PassResult::rosCommandIdCannback, this);
        sub_door_id_ = ros_nh_.subscribe("/automatic_door/doorId", 1,
                                &PassResult::rosDoorIdCallback, this);

        // timer_ = ros_nh_.createTimer(ros::Duration(info_rate_), 
        //                         &RobotState::goThroughDoor, this);

    }

private:
    ros::NodeHandle ros_nh_;
    ros::Subscriber sub_pass_result_;
    ros::Subscriber sub_command_id_;
    ros::Subscriber sub_door_id_;
    std_msgs::String door_id_;
    std_msgs::String command_id_;
    ros::Timer timer_;
    ComderInfoPayload<PassResultinfoData> topic_info_;

    // void goThroughDoor(const ros::TimerEvent& e){
        
    // }

    void rosDoorIdCallback(const std_msgs::String& msg){
        door_id_ = msg;
    }

    void rosCommandIdCannback(const std_msgs::String& msg){
        command_id_ = msg;
    }

    void rosPassResultCallback(const std_msgs::Bool& msg){
        topic_info_.clientId = getRobotID();
        topic_info_.timestamp = getUnixTimeStamp();
        topic_info_.data.isSucceed = msg.data;
        topic_info_.data.doorId = door_id_.data;
        topic_info_.data.commandId = command_id_.data;
        publishTopic(getInfoTopicName(), topic_info_);
    }
};//end struct PassResult

}//end namespace zros

#endif /*__COMDER_PASS_RESULT_H__*/