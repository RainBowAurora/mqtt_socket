/**
 * @file function_set_site.hpp
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-05
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __FUNCTION_SET_SITE_HPP__
#define __FUNCTION_SET_SITE_HPP__

 
#include "../../rdms_topic_factory.h"
#include "../../rdms_topic_base.h"

struct FunctionSetSiteReqData{
    std::string siteId;
};
DEFINE_STRUCT_SCHEMA(FunctionSetSiteReqData,\
                    DEFINE_STRUCT_FIELD(siteId, "siteId"));
DEFINE_STRUCT_REQUEST_FIELD(FunctionSetSiteReqData);

namespace zros{

class FunctionSetSite final: public RdmsTopicBase{
public:
    FunctionSetSite(): ros_nh_(){

    }

    void init() override{
        RdmsTopicBase::topicInit("function", "set-site");
        subscribeTopic(getReqTopicName(), 
                        &FunctionSetSite::setSiteCallback, this);
        
    }

private:
    ros::NodeHandle ros_nh_; //ros node handle
    
    MqttRequestPayload<FunctionSetSiteReqData> topic_resquest_; 
    MqttResponePayload<emptyData> topic_respone_;

    void setSiteCallback(const std::string& msg){
        parseMessage(msg, topic_resquest_);
        std::string correlationId = topic_resquest_.correlationId;
        std::string clientId = topic_resquest_.clientId;
        int16_t timestamp = topic_resquest_.timestamp;
        std::string siteId = topic_resquest_.data.siteId;
        std::string current_siteId = getSiteID();
        topic_respone_.correlationId = correlationId;
        topic_respone_.clientId = getRobotID();
        topic_respone_.timestamp = getUnixTimeStamp();
        topic_respone_.data = {};
        
        ros::param::set("/site_id", siteId);

        topic_respone_.result = "SUCCESS";
        topic_respone_.message = "I changed siteId from site " + current_siteId + "to" + siteId;


        publishTopic(getResTopicName(), topic_respone_);
    } 

}; //end class FunctionSetSite

REGIST_MQTT_TOPIC(FunctionSetSite, FunctionSetSite);

} //end namespace zros

#endif /*__FUNCTION_SET_SITE_HPP__*/