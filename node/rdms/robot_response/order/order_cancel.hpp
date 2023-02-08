/**
 * @file order_cancel.h
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __ORDER_CANCEL_H__
#define __ORDER_CANCEL_H__

#include "../../rdms_topic_factory.h"
#include "../../rdms_topic_base.h"
#include <std_msgs/String.h>
struct OrderCancelReqData{
    int deliveryId;
    std::string orderNumber;
};
DEFINE_STRUCT_SCHEMA(OrderCancelReqData,\
                    DEFINE_STRUCT_FIELD(deliveryId, "deliveryId"),\
                    DEFINE_STRUCT_FIELD(orderNumber, "orderNumber"));
DEFINE_STRUCT_REQUEST_FIELD(OrderCancelReqData);

namespace zros{

class OrderCancel final: public RdmsTopicBase{
public:
    OrderCancel(): ros_nh_(){

    }

    void init() override{
        RdmsTopicBase::topicInit("order", "cancel");
        subscribeTopic(getReqTopicName(), 
                        &OrderCancel::orderCancelCallback, this);
        pub_destination_ = ros_nh_.advertise<std_msgs::String>("/task_destination", 1, true);
        pub_mission_cancel_ = ros_nh_.advertise<std_msgs::String>("/mission_cancel", 1, true);

    }

private:
    ros::NodeHandle ros_nh_; //ros node handle
    ros::Publisher pub_destination_;
    ros::Publisher pub_mission_cancel_;
    MqttRequestPayload<OrderCancelReqData> topic_resquest_; 
    MqttResponePayload<emptyData> topic_respone_;

    void orderCancelCallback(const std::string& msg){
        parseMessage(msg, topic_resquest_);
        std::string correlationId = topic_resquest_.correlationId;
        std::string orderNumber = topic_resquest_.data.orderNumber;
        int delivery_id = topic_resquest_.data.deliveryId;

        // std_msgs::String deliveryId;
        // deliveryId.data = delivery_id;
        // pub_mission_cancel_.publish(deliveryId);

        // std_msgs::String temp_destination;
        // temp_destination.data = "cancel";
        // pub_destination_.publish(temp_destination);

        topic_respone_.correlationId = correlationId;
        topic_respone_.clientId = getRobotID();
        topic_respone_.timestamp = getUnixTimeStamp();
        topic_respone_.result = "SUCCESS";
        topic_respone_.message = "stopped the delivery for the canceled order";
        topic_respone_.data = {};
        publishTopic(getResTopicName(), topic_respone_);
    }

}; //end class OrderCancel

REGIST_MQTT_TOPIC(OrderCancel, OrderCancel);

} //end namespace zros

#endif /*__ORDER_CANCEL_H__*/