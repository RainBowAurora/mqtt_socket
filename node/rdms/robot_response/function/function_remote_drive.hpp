/**
 * @file function_remote_drive.hpp
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __FUNCTION_REMOTE_DRIVE_HPP__
#define __FUNCTION_REMOTE_DRIVE_HPP__

 
#include <std_msgs/Int32.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include "../../rdms_topic_factory.h"
#include "../../rdms_topic_base.h"
#include <cstdlib>

struct FunctionRemoteDriveReqData{
    std::string drivingSpeed;
    std::string spinSpeed;
};
DEFINE_STRUCT_SCHEMA(FunctionRemoteDriveReqData,\
                    DEFINE_STRUCT_FIELD(drivingSpeed, "drivingSpeed"),\
                    DEFINE_STRUCT_FIELD(spinSpeed, "spinSpeed"));
DEFINE_STRUCT_REQUEST_FIELD(FunctionRemoteDriveReqData);

namespace zros{

class FunctionRemoteDrive final: public RdmsTopicBase{
public:
    FunctionRemoteDrive(): ros_nh_(){
        
    }

    void init() override{
        RdmsTopicBase::topicInit("function", "remote-drive");
        subscribeTopic(getReqTopicName(), 
                        &FunctionRemoteDrive::remoteDriveCallback, this);

        pub_cmd_vel_ = ros_nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
        
        sub_current_pose_ = ros_nh_.subscribe("/current_pose",10 ,
                        &FunctionRemoteDrive::RobotCurrentPoseCallback, this);

        sub_current_pose_ = ros_nh_.subscribe("/start_flag", 10,
                        &FunctionRemoteDrive::RobotStartFlagCallback, this);
    } 

private:
    ros::NodeHandle ros_nh_; //ros node handle
    ros::Publisher pub_cmd_vel_;
    ros::Subscriber sub_current_pose_;
    std::string robot_current_pose_ = "(unknow, unknow)";
    int32_t start_flag_;
    MqttRequestPayload<FunctionRemoteDriveReqData> topic_resquest_; 
    MqttResponePayload<emptyData> topic_respone_;

    void RobotCurrentPoseCallback(const geometry_msgs::PoseStamped& msg){
        std::string pose_x = std::to_string(msg.pose.position.x);
        std::string pose_y = std::to_string(msg.pose.position.y); 
        robot_current_pose_ = "(" + pose_x + "," + pose_y + ")";
    }

    void RobotStartFlagCallback(const std_msgs::Int32& msg){
        start_flag_ = msg.data;
    }

    void remoteDriveCallback(const std::string& msg){
        parseMessage(msg, topic_resquest_);
        std::string correlationId = topic_resquest_.correlationId;
        std::string clientId = topic_resquest_.clientId;
        int16_t timestamp = topic_resquest_.timestamp;
        double drivingspeed = atof(topic_resquest_.data.drivingSpeed.c_str());
        double spinSpeed = -atof(topic_resquest_.data.spinSpeed.c_str());

        geometry_msgs::Twist tmp_cmd;
        tmp_cmd.linear.x = drivingspeed;
        tmp_cmd.angular.z = spinSpeed;
        pub_cmd_vel_.publish(tmp_cmd);

        topic_respone_.correlationId = correlationId;
        topic_respone_.clientId = getRobotID();
        topic_respone_.timestamp = getUnixTimeStamp();
        topic_respone_.data = {};

        if(start_flag_== 0){
            topic_respone_.result = "SUCCESS";
            topic_respone_.message = "move by " + robot_current_pose_;
        }else if(start_flag_== 1){
            topic_respone_.result = "FAIL";
            topic_respone_.message = "the current mode is not manual control!";
        }

        publishTopic(getResTopicName(), topic_respone_);
    }
}; //end class FunctionRemoteDrive

REGIST_MQTT_TOPIC(FunctionRemoteDrive, FunctionRemoteDrive);

} //end namespace zros

#endif /*__FUNCTION_REMOTE_DRIVE_HPP__*/