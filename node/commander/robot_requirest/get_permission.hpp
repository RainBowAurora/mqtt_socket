#ifndef __COMDER_PERMISSION_H__
#define __COMDER_PERMISSION_H__

#include <ros/ros.h>
#include "../comder_topic_factory.h"
#include "../comder_topic_base.h"
#include <std_msgs/String.h>
                    

struct GetPermissionReqData{
    std::string doorId;
};//end struct GetPermissionReqData
DEFINE_STRUCT_SCHEMA(GetPermissionReqData,\
    DEFINE_STRUCT_FIELD(doorId, "doorId"));
CMDER_DEFINE_STRUCT_REQ_FIELD(GetPermissionReqData);

struct GetPermissionResData{
    std::string commandId;
}; //end GetPermissionResData
DEFINE_STRUCT_SCHEMA(GetPermissionResData,\
    DEFINE_STRUCT_FIELD(commandId, "commandId"));
CMDER_DEFINE_STRUCT_RES_FIELD(GetPermissionResData);

namespace zros{

class GetPermission final: public ComderTopicBase{
public:
    GetPermission(): ros_nh_(){

    }
    void init() override{
        GetPermission::topicInit("get-permission");
        subscribeTopic(getResTopicName(),
                                    &GetPermission::mqttGetperMissionCallback, this);
        sub_auto_door_permission_ = ros_nh_.subscribe("/automatic_door/get_permission", 1,
                                   &GetPermission::rosGetPermissionCallback, this);
        pub_permission_ = ros_nh_.advertise<std_msgs::String>("automatic_door/permission", 1);
    }

private:
    ros::NodeHandle ros_nh_;
    ros::Subscriber sub_auto_door_permission_;
    ros::Publisher pub_permission_;
    std::string door_id_;
    std::string command_id_;
    ComderReqPlayload<GetPermissionReqData> topic_request_;
    ComderResPlayPoad<GetPermissionResData> topic_respone_;

    void mqttGetperMissionCallback(const std::string& msg){
        parseMessage(msg, topic_respone_);
        command_id_ = topic_respone_.data.commandId;
        std_msgs::String temp_command_id;

        if(topic_respone_.result == "SUCCESS"){
            temp_command_id.data = command_id_;
        }else{
            temp_command_id.data = "ERROR";
            std::cout << "[ERROR] get permission error reason:"  << topic_respone_.message << std::endl;
        }
        pub_permission_.publish(temp_command_id);
    }

    void rosGetPermissionCallback(const std_msgs::String& msg){
        door_id_ = msg.data;
        topic_request_.correlationId = generateUUID();
        topic_request_.clientId = getRobotID();
        topic_request_.timestamp = getUnixTimeStamp();
        topic_request_.data.doorId = door_id_;
        publishTopic(getReqTopicName(), topic_request_);
    }

};//end class GetPermission

REGIST_COMDER_TOPIC(GetPermission, GetPermission);

}//end namespace zros


#endif /*__COMDER_PERMISSION_H__*/