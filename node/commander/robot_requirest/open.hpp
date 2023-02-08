#ifndef __COMDER_OPEN_H__
#define __COMDER_OPEN_H__

#include <std_msgs/String.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include "../comder_topic_factory.h"
#include "../comder_topic_base.h"

struct OpenReqData{
    std::string doorId;
    std::string commandId;
}; //end struct OpenReqData
DEFINE_STRUCT_SCHEMA(OpenReqData,\
    DEFINE_STRUCT_FIELD(doorId, "doorId"),\
    DEFINE_STRUCT_FIELD(commandId, "commandId"));
CMDER_DEFINE_STRUCT_REQ_FIELD(OpenReqData);

struct OpenResData{
    std::string commandId;
}; //end struct OpenResData
DEFINE_STRUCT_SCHEMA(OpenResData,\
    DEFINE_STRUCT_FIELD(commandId, "commandId"));
CMDER_DEFINE_STRUCT_RES_FIELD(OpenResData);

namespace zros{

class OpenDoor final: public ComderTopicBase{
public:
    OpenDoor(): ros_nh_(){

    }
    void init() override{
        ComderTopicBase::topicInit("open");
        subscribeTopic(getResTopicName(),
                        &OpenDoor::mqttOpenResultCallback, this);

        sub_open_door_ = ros_nh_.subscribe("/automatic_door/retryOpen", 1,\
                               &OpenDoor::rosOpenDoorCallback, this);
        sub_door_id_ = ros_nh_.subscribe("/automatic_door/doorId", 1,\
                               &OpenDoor::rosDoorIdCallback, this);
        sub_command_id_ = ros_nh_.subscribe("/automatic_door/permission", 1,\
                               &OpenDoor::rosMissionIdCallback, this);

        sub_door_nearby_ = ros_nh_.subscribe("/automatic_door/doorNearby", 1,\
                               &OpenDoor::rosDoorNearby, this);

        pub_pass_result_ = ros_nh_.advertise<std_msgs::Bool>("/automatic_door/passResult", 1);

        pub_get_permission_ = ros_nh_.advertise<std_msgs::String>("/automatic_door/get_permission", 1);

        pub_start_flag_ = ros_nh_.advertise<std_msgs::Int32>("/start_flag", 1);
    }
private:
    ros::NodeHandle ros_nh_;
    ros::Subscriber sub_door_id_;
    ros::Subscriber sub_open_door_;
    ros::Subscriber sub_command_id_;
    ros::Subscriber sub_door_nearby_;
    ros::Publisher pub_pass_result_;
    ros::Publisher pub_get_permission_;
    ros::Publisher pub_start_flag_;
    std_msgs::Bool door_nearby_;
    std_msgs::String door_id_;
    std_msgs::String command_id_;
    ComderReqPlayload<OpenReqData> topic_request_;
    ComderResPlayPoad<OpenResData> topic_respone_;

    void rosDoorNearby(const std_msgs::Bool& msg){
        door_nearby_ = msg;
    }

    void rosOpenDoorCallback(const std_msgs::Bool& msg){
        if(msg.data == true && !door_id_.data.empty() && door_nearby_.data){
            pub_get_permission_.publish(door_id_); 
        }else{
            std_msgs::Bool result;
            result.data = false;
            pub_pass_result_.publish(result);          
        }
    }

    void rosDoorIdCallback(const std_msgs::String& msg){
        door_id_ = msg;
        if(door_nearby_.data){
            std_msgs::Int32 temp_start_flag;
            temp_start_flag.data = 0;
            pub_start_flag_.publish(temp_start_flag);
            pub_get_permission_.publish(door_id_);
        }
    }

    void rosMissionIdCallback(const std_msgs::String& msg){
        command_id_ = msg;
        topic_request_.correlationId = generateUUID();
        topic_request_.clientId = getRobotID();
        topic_request_.timestamp = getUnixTimeStamp();
        topic_request_.data.doorId = door_id_.data;
        topic_request_.data.commandId = command_id_.data;
        publishTopic(getReqTopicName(), topic_request_);
    }
    void mqttOpenResultCallback(const std::string& msg){
        parseMessage(msg, topic_respone_);
        std_msgs::Bool result;
        result.data = (topic_respone_.result == "SUCCESS"? true: false);
        pub_pass_result_.publish(result);
    }

};//end class OpenDoor
REGIST_COMDER_TOPIC(OpenDoor, OpenDoor);

}//end namespace zros

#endif /*__COMDER_OPEN_H__*/