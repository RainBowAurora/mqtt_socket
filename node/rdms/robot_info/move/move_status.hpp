/**
 * @file move_status.hpp
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __MOVE_STATUS_HPP__
#define __MOVE_STATUS_HPP__

 
#include <std_msgs/String.h>

#include "../../rdms_topic_factory.h"
#include "../../rdms_topic_base.h"
#include <zr_msgs/driving_status.h>

struct MoveStatusData{
    std::string status;
};
DEFINE_STRUCT_SCHEMA(MoveStatusData, \
                    DEFINE_STRUCT_FIELD(status, "status"));
                    
DEFINE_STRUCT_INFO_FIELD(MoveStatusData);

namespace zros{

class MoveStatus final: public RdmsTopicBase{
public:
    MoveStatus(): ros_nh_(){

    } 

    void init() override{
        RdmsTopicBase::topicInit("move", "status");    
                                        
        sub_move_status_ = ros_nh_.subscribe("/driving_info", 1, 
                                        &MoveStatus::rosMoveStatusCallback, this);
    }

private:
    ros::NodeHandle ros_nh_; //ros node handle
    ros::Subscriber sub_move_status_;
    double info_rate_;
    MqttInfoPayload<MoveStatusData> info_playload_;

    void rosMoveStatusCallback(const zr_msgs::driving_status& msg){
        uploadMessage(msg.status);
    }

    void uploadMessage(const std::string& move_status){
        info_playload_.clientId = getRobotID();
        info_playload_.timestamp = getUnixTimeStamp();
        info_playload_.data.status = move_status;
        publishTopic(getInfoTopicName(), info_playload_);
    }

}; //end class MoveStatus

REGIST_MQTT_TOPIC(MoveStatus, MoveStatus);

} //end namespace zros

#endif /*__MOVE_STATUS_HPP__*/