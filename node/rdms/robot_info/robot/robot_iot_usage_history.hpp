#ifndef __ROBOT_IOT_USAGE_HISTORY_H__
#define __ROBOT_IOT_USAGE_HISTORY_H__

#include <std_msgs/String.h>
#include <zr_msgs/mission.h>
#include <zr_msgs/task.h>
#include <zr_msgs/order_info.h>
#include "../../rdms_topic_factory.h"
#include "../../rdms_topic_base.h"


struct RobotIotUsageHistoryData{
    std::string commandType;
    std::string commandId;
    std::string referenceType;
    std::string referenceKey;
};//end struct RobotIotUsageHistoryData
DEFINE_STRUCT_SCHEMA(RobotIotUsageHistoryData, \
                    DEFINE_STRUCT_FIELD(commandType, "commandType"),\
                    DEFINE_STRUCT_FIELD(commandId, "commandId"),\
                    DEFINE_STRUCT_FIELD(referenceType, "referenceType"),\
                    DEFINE_STRUCT_FIELD(referenceKey, "referenceKey"));             
DEFINE_STRUCT_INFO_FIELD(RobotIotUsageHistoryData);

namespace zros{

class RobotIotUsageHistory final: public RdmsTopicBase{
public:
    RobotIotUsageHistory(): ros_nh_(){

    }

    void init() override{
        RdmsTopicBase::topicInit("robot", "iot-usage-history");     
        sub_pass_result_ = ros_nh_.subscribe("/automatic_door/passResult", 1,
                        &RobotIotUsageHistory::rosPassResultCallback, this);
        sub_command_id_ = ros_nh_.subscribe("/automatic_door/permission", 1,
                        &RobotIotUsageHistory::rosCommandIdCannback, this);
        sub_door_id_ = ros_nh_.subscribe("/automatic_door/doorId", 1,
                        &RobotIotUsageHistory::rosDoorIdCallback, this);
        sub_delivery_mission_ = ros_nh_.subscribe("/mission", 1,
                        &RobotIotUsageHistory::rosMissionCallback, this);
    }

private:
    ros::NodeHandle ros_nh_; //ros node handle
    ros::Subscriber sub_pass_result_;
    ros::Subscriber sub_command_id_;
    ros::Subscriber sub_door_id_;
    ros::Subscriber sub_delivery_mission_;
    
    std_msgs::String auto_door_id_;
    std_msgs::String auto_door_commandid_;
    zr_msgs::mission mission_;

    void rosMissionCallback(const zr_msgs::mission& msg){
        mission_ = msg;
    }

    void rosPassResultCallback(const std_msgs::Bool& msg){
        MqttInfoPayload<RobotIotUsageHistoryData> robot_iot_playload;
        robot_iot_playload.clientId = getRobotID();
        robot_iot_playload.timestamp = getUnixTimeStamp();
        robot_iot_playload.data.commandType = "AUTOMATIC_DOORS";
        robot_iot_playload.data.commandId = auto_door_commandid_.data;
        robot_iot_playload.data.referenceType = "UNDEFINED";
        robot_iot_playload.data.referenceKey = "UNDEFINED";

        for(auto task: mission_.task_array){
            robot_iot_playload.data.referenceType = task.task_type;
            robot_iot_playload.data.referenceKey = task.task_id;
        }
        publishTopic(getInfoTopicName(), robot_iot_playload);
    }

    void rosCommandIdCannback(const std_msgs::String& msg){
        auto_door_commandid_ = msg;
    }

    void rosDoorIdCallback(const std_msgs::String& msg){
        auto_door_id_ = msg;
    }

}; //end class RobotIotUsageHistory

REGIST_MQTT_TOPIC(RobotIotUsageHistory, RobotIotUsageHistory);

} //end namespace zros

#endif /*__ROBOT_IOT_USAGE_HISTORY_H__*/