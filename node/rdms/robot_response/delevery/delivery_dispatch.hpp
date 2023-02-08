/**
 * @file delivery_dispatch.hpp
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __DELIVERY_DISPATCH_HPP__
#define __DELIVERY_DISPATCH_HPP__

#include "../../rdms_topic_factory.h"
#include "../../rdms_topic_base.h"
#include <zr_msgs/mission.h>
#include <zr_msgs/task.h>
#include <zr_msgs/order_info.h>

struct DeliveryDispitchReqData{
    int deliveryId;
    std::string orderNumber;
    std::string shop;
    std::string destination;
}; //end struct DeliveryDispitchReqData

DEFINE_STRUCT_SCHEMA(DeliveryDispitchReqData,\
                    DEFINE_STRUCT_FIELD(deliveryId, "deliveryId"),\
                    DEFINE_STRUCT_FIELD(orderNumber, "orderNumber"),\
                    DEFINE_STRUCT_FIELD(shop, "shop"),\
                    DEFINE_STRUCT_FIELD(destination, "destination"));
DEFINE_STRUCT_REQUEST_FIELD(DeliveryDispitchReqData);

namespace zros{

class DeliveryDispitch final: public RdmsTopicBase{
public:
    DeliveryDispitch():ros_nh_(){

    }

    void init() override{
        RdmsTopicBase::topicInit("delivery", "dispatch");
        subscribeTopic(getReqTopicName(), 
                                &DeliveryDispitch::dispatchMessageCallback, this);
        pub_delivery_mission_ = ros_nh_.advertise<zr_msgs::mission>("/mission", 1, true);

        pub_destination_ = ros_nh_.advertise<std_msgs::String>("/task_destination", 1, true);
    }

private:
    ros::NodeHandle ros_nh_; //ros node handle
    ros::Publisher pub_delivery_mission_;
    ros::Publisher pub_destination_;
    MqttRequestPayload<DeliveryDispitchReqData> topic_resquest_; 
    MqttResponePayload<emptyData> topic_respone_;

    void dispatchMessageCallback(const std::string& msg){
        parseMessage(msg, topic_resquest_);
        topic_respone_.correlationId = topic_resquest_.correlationId;

        int delivery_id =  topic_resquest_.data.deliveryId;
        std::string order_number = topic_resquest_.data.orderNumber;
        std::string shop = topic_resquest_.data.shop;
        std::string destination = topic_resquest_.data.destination;

        zr_msgs::mission temp_mission;
        zr_msgs::task temp_delivery_task;
        temp_mission.mission_id = delivery_id;
        temp_delivery_task.task_type = "DELIVERY";
        temp_delivery_task.task_id = std::to_string(delivery_id);
        temp_delivery_task.order.order_id = delivery_id;
        temp_delivery_task.order.store_id = shop;
        temp_delivery_task.order.dest_id = destination;
        temp_delivery_task.dest_id = destination;
        temp_delivery_task.priority = 0;
        temp_mission.task_array.push_back(temp_delivery_task);
        pub_delivery_mission_.publish(temp_mission);

        std_msgs::String temp_destination;
        temp_destination.data = destination;
        pub_destination_.publish(temp_destination);

        topic_respone_.clientId = getRobotID();
        topic_respone_.timestamp = getUnixTimeStamp();
        topic_respone_.result = "SUCCESS";
        topic_respone_.message = "create dispatch success";
        topic_respone_.data = {};
        publishTopic(getResTopicName(), topic_respone_);
    }

}; //end class DeliveryDispitch

REGIST_MQTT_TOPIC(DeliveryDispitch, DeliveryDispitch);

} //end namespace zros

#endif /*__DELIVERY_DISPATCH_HPP__*/