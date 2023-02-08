/**
 * @file move_cancel_back.hpp
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __MOVE_CANCEL_BACK_HPP__
#define __MOVE_CANCEL_BACK_HPP__

#include <std_msgs/Int32.h>
#include "../../rdms_topic_factory.h"
#include "../../rdms_topic_base.h"

struct MoveCancelBackReqData{
    int moveId;
}; 
DEFINE_STRUCT_SCHEMA(MoveCancelBackReqData, \
                    DEFINE_STRUCT_FIELD(moveId, "moveId"));    
DEFINE_STRUCT_REQUEST_FIELD(MoveCancelBackReqData);

namespace zros{

class MoveCancelBack final: public RdmsTopicBase{
public:
    MoveCancelBack(): ros_nh_(){

    }

    void init() override{
        RdmsTopicBase::topicInit("move", "cancel-back");
        subscribeTopic(getResTopicName(), 
                                &MoveCancelBack::mqttCnacelMoveCallback, this);        

        sub_cancel_move_ = ros_nh_.subscribe("/cancel_move", 1, 
                                &MoveCancelBack::rosCancelMoveCallback, this);
    }

private:
    ros::NodeHandle ros_nh_; //ros node handle
    ros::Subscriber sub_cancel_move_;
    MqttRequestPayload<MoveCancelBackReqData> topic_request_;
    MqttResponePayload<emptyData> topic_respone_;

    void pubCancelMove(const int32_t move_id){
        topic_request_.correlationId = generateUUID();
        topic_request_.clientId = getRobotID();
        topic_request_.timestamp = getUnixTimeStamp();
        topic_request_.data.moveId = move_id;
        publishTopic(getReqTopicName(), topic_request_);
    }

    void rosCancelMoveCallback(const std_msgs::Int32& msg){
        pubCancelMove(msg.data);
    }

    void mqttCnacelMoveCallback(const std::string& msg){
        parseMessage(msg, topic_respone_);
    }

}; //end class MoveCancelBack

REGIST_MQTT_TOPIC(MoveCancelBack, MoveCancelBack);

} //end namespace zros


#endif /*__MOVE_CANCEL_BACK_HPP__*/