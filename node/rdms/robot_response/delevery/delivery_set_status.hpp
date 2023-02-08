/**
 * @file delivery_set_status.hpp
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __DELIVERY_SET_STATUS_HPP__
#define __DELIVERY_SET_STATUS_HPP__

 
#include "../../rdms_topic_factory.h"
#include "../../rdms_topic_base.h"
#include <std_msgs/String.h>

struct DeliverySetStatusReqData{
    int deliveryId;
    std::string status;
}; //end struct DeliverySetStatusReqData
DEFINE_STRUCT_SCHEMA(DeliverySetStatusReqData,\
                    DEFINE_STRUCT_FIELD(deliveryId, "deliveryId"),\
                    DEFINE_STRUCT_FIELD(status, "status"));
DEFINE_STRUCT_REQUEST_FIELD(DeliverySetStatusReqData);

namespace zros{

class DeliverySetStatus final: public RdmsTopicBase{
public:
    DeliverySetStatus(): ros_nh_(){

    }

    void init() override{
        RdmsTopicBase::topicInit("delivery", "set-status");
        subscribeTopic(getReqTopicName(), 
                                &DeliverySetStatus::setStatusCallback, this);        
        pub_force_state_ = ros_nh_.advertise<std_msgs::String>("/force_state", 1);
    }
private:
    ros::NodeHandle ros_nh_; //ros node handle
    ros::Publisher pub_force_state_;
    std_msgs::String force_state_;
    MqttRequestPayload<DeliverySetStatusReqData> topic_resquest_; 
    MqttResponePayload<emptyData> topic_respone_; 

    void setStatusCallback(const std::string& msg){
        static std::string last_status = "WAIT_ALLOCATE";
        parseMessage(msg, topic_resquest_);
        std::string status = topic_resquest_.data.status;

        topic_respone_.correlationId = topic_resquest_.correlationId;
        topic_respone_.clientId = getRobotID();
        topic_respone_.timestamp = getUnixTimeStamp();

        force_state_.data = status;
        pub_force_state_.publish(force_state_);

        if(last_status != status){
            topic_respone_.result = "SUCCESS";
            topic_respone_.message = "changed the delivery status from " + \
                                        last_status + "to" + status;
        }else{
            topic_respone_.result = "FAIL";
            topic_respone_.message = "cannot changed the delivery status from " + \
                                        last_status + "to" + status;        
        }
        topic_respone_.data = {};
        publishTopic(getResTopicName(), topic_respone_);
        last_status = status;
    }

}; //end class DeliverySetStatus

REGIST_MQTT_TOPIC(DeliverySetStatus, DeliverySetStatus);

} //end namespace zros

#endif /*__DELIVERY_SET_STATUS_HPP__*/