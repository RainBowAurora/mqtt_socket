/**
 * @file delivery_cancel_back.hpp
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __DELIVERY_CANCEL_BACK_HPP__
#define __DELIVERY_CANCEL_BACK_HPP__

#include <std_msgs/Int32.h>

#include "../../rdms_topic_factory.h"
#include "../../rdms_topic_base.h"
#include <zr_msgs/delivery_cancel.h>

struct DeliveryCancelBackReqData{
    int deliveryId;
};

DEFINE_STRUCT_SCHEMA(DeliveryCancelBackReqData, \
                    DEFINE_STRUCT_FIELD(deliveryId, "deliveryId"));
                    
DEFINE_STRUCT_REQUEST_FIELD(DeliveryCancelBackReqData);


namespace zros{

class DeliveryCancelBack final: public RdmsTopicBase{
public:
    DeliveryCancelBack(): ros_nh_(){

    }

    void init() override{
        RdmsTopicBase::topicInit("delivery", "cancel-back");
        subscribeTopic(getResTopicName(), 
                                &DeliveryCancelBack::mqttCancelDeliveryCallback, this);   

        sub_cancel_delivery_ = ros_nh_.subscribe("/driving_cancel", 1, 
                                &DeliveryCancelBack::rosCancelDeliveryCallback, this);
    }

private:
    ros::NodeHandle ros_nh_; //ros node handle
    ros::Subscriber sub_cancel_delivery_;
    MqttRequestPayload<DeliveryCancelBackReqData> topic_request_;
    MqttResponePayload<emptyData> topic_respone_;

    void pubDeliveryCancel(const int32_t delivery_id){
        topic_request_.correlationId = generateUUID();
        topic_request_.clientId = getRobotID();
        topic_request_.timestamp = getUnixTimeStamp();
        topic_request_.data.deliveryId = delivery_id;
        publishTopic(getReqTopicName(),topic_request_);
    }

    void rosCancelDeliveryCallback(const zr_msgs::delivery_cancel& msg){
            pubDeliveryCancel(msg.orderId);
    }

    void mqttCancelDeliveryCallback(const std::string& msg){
            parseMessage(msg, topic_respone_);
    }

}; //end class DeliveryCancelBack

REGIST_MQTT_TOPIC(DeliveryCancelBack, DeliveryCancelBack);

} //end namespace zros

#endif /*__DELIVERY_CANCEL_BACK_HPP__*/