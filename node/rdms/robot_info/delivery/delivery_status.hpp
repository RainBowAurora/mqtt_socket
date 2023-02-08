/**
 * @file delivery_status.hpp
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __DELIVERY_STATUS_HPP__
#define __DELIVERY_STATUS_HPP__

#include "../../rdms_topic_base.h"
#include "../../rdms_topic_factory.h"
#include <std_msgs/String.h>
#include <zr_msgs/delivery_status.h>

struct DeliveryStatusData{
    std::string status;
};//end struct DeliveryStatusData

DEFINE_STRUCT_SCHEMA(DeliveryStatusData, \
                    DEFINE_STRUCT_FIELD(status, "status"));
DEFINE_STRUCT_INFO_FIELD(DeliveryStatusData);

struct DeliveryStatusIdData{
    int deliveryId;
    std::string status;
};//end struct DeliveryStatusIdData
DEFINE_STRUCT_SCHEMA(DeliveryStatusIdData, \
                    DEFINE_STRUCT_FIELD(deliveryId, "deliveryId"),\
                    DEFINE_STRUCT_FIELD(status, "status"));
DEFINE_STRUCT_INFO_FIELD(DeliveryStatusIdData);


struct DeliveryStatusRecallData{
    int deliveryId;
    std::string status;
    std::string recallReason;
};//end struct DeliveryStatusRecallData
DEFINE_STRUCT_SCHEMA(DeliveryStatusRecallData, \
                    DEFINE_STRUCT_FIELD(deliveryId, "deliveryId"),\
                    DEFINE_STRUCT_FIELD(status, "status"),\
                    DEFINE_STRUCT_FIELD(recallReason, "recallReason"));
DEFINE_STRUCT_INFO_FIELD(DeliveryStatusRecallData);

namespace zros{

class DeliveryStatus final: public RdmsTopicBase{
public:
    DeliveryStatus(): ros_nh_(){

    }

    void init() override{
        RdmsTopicBase::topicInit("delivery", "status");
        sub_delivery_status_ = ros_nh_.subscribe("/delivery_info", 1, 
                                        &DeliveryStatus::rosDeliveryStatus, this);
    }

private:
    ros::NodeHandle ros_nh_; //ros node handle
    ros::Subscriber sub_delivery_status_;
    MqttInfoPayload<DeliveryStatusData> info_playload_;
    MqttInfoPayload<DeliveryStatusIdData> info_delivery_id_playload_;
    MqttInfoPayload<DeliveryStatusRecallData> recall_reason_playload_;
    
    void rosDeliveryStatus(const zr_msgs::delivery_status& msg){
        uploadMessage(msg);
    }

    void uploadMessage(const zr_msgs::delivery_status& msg){
        recall_reason_playload_.clientId = info_delivery_id_playload_.clientId \
                                        = info_playload_.clientId = getRobotID();

        recall_reason_playload_.timestamp = info_delivery_id_playload_.timestamp \
                                        = info_playload_.timestamp = getUnixTimeStamp();

        recall_reason_playload_.data.status = info_delivery_id_playload_.data.status \
                                        = info_playload_.data.status = msg.status;

        
        info_delivery_id_playload_.data.deliveryId = msg.orderId;

        
        if(msg.status == "WAIT_ALLOCATE"){
            publishTopic(getInfoTopicName(), info_playload_);
        }else if(msg.status == "RECALL"){
            publishTopic(getInfoTopicName(), recall_reason_playload_);
        }else{
            publishTopic(getInfoTopicName(), info_delivery_id_playload_);
        }
    }


}; //end class DeliveryStatus

REGIST_MQTT_TOPIC(DeliveryStatus,DeliveryStatus);

} //end namespace zros

#endif /*__DELIVERY_STATUS_HPP__*/