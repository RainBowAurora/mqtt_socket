/**
 * @file delivery_cancel.hpp
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __DELIVERY_CANCEL_HPP__
#define __DELIVERY_CANCEL_HPP__

 
#include "../../rdms_topic_factory.h"
#include "../../rdms_topic_base.h"
#include <std_msgs/String.h>

struct DeliveryCancelReqData{
    int deliveryId;
};//end struct DeliveryCancelReqData
DEFINE_STRUCT_SCHEMA(DeliveryCancelReqData,
                    DEFINE_STRUCT_FIELD(deliveryId, "deliveryId"));
DEFINE_STRUCT_REQUEST_FIELD(DeliveryCancelReqData);

namespace zros{

class DeliveryCancel final: public RdmsTopicBase{
public:
    DeliveryCancel():ros_nh_(){

    }

    void init() override{
        RdmsTopicBase::topicInit("delivery", "cancel");
        subscribeTopic(getReqTopicName(), 
                                &DeliveryCancel::cancelMessageCallback, this);

        pub_destination_ = ros_nh_.advertise<std_msgs::String>("/task_destination", 1, true);
        pub_mission_cancel_ = ros_nh_.advertise<std_msgs::String>("/mission_cancel", 1, true);
    }

public:
    ros::NodeHandle ros_nh_; //ros node handle
    ros::Publisher pub_destination_;
    ros::Publisher pub_mission_cancel_;
    MqttRequestPayload<DeliveryCancelReqData> topic_resquest_; 
    MqttResponePayload<emptyData> topic_respone_;
    void cancelMessageCallback(const std::string& msg){
        parseMessage(msg, topic_resquest_);

        std::string delivery_id = std::to_string(topic_resquest_.data.deliveryId);

        topic_respone_.correlationId = topic_resquest_.correlationId;
        topic_respone_.clientId = getRobotID(); 
        topic_respone_.timestamp = getUnixTimeStamp();
        topic_respone_.result = "SUCCESS";
        topic_respone_.message = "canceled the delivery successfully";

        std_msgs::String deliveryId;
        deliveryId.data = delivery_id;
        pub_mission_cancel_.publish(deliveryId);

        std_msgs::String temp_destination;
        temp_destination.data = "cancel";
        pub_destination_.publish(temp_destination);

        topic_respone_.data = {};
        publishTopic(getResTopicName(), topic_respone_);
    }
    
}; //end class DeliveryCancel

REGIST_MQTT_TOPIC(DeliveryCancel, DeliveryCancel);

} //end namespace zros

#endif /*__DELIVERY_CANCEL_HPP__*/