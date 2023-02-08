#include "rdms_topic_base.h"
#include <chrono>
#include <string>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>

namespace zros{

std::unique_ptr<Mqtt> RdmsTopicBase::rdms_mqtt_ = nullptr;

RdmsTopicBase::RdmsTopicBase(): ros_nh_(), ros_param_("~"){
    rosParamInit();
    switchBroker();
}

RdmsTopicBase::~RdmsTopicBase()
{
    for(auto element: sub_topic_vec_){ //unsubscribe topic
        unsubscribe(element);
    }
}

void RdmsTopicBase::switchBroker(){
    int   RDMS_PORT = 0;
    std::string RDMS_HOST = "undefined";
    std::string RDMS_USERNAME = "undefined";
    std::string RDMS_PASSWORD = "undefined";

    if("Dev" == mqtt_broker_){
        RDMS_HOST = "****";
        RDMS_PORT = 8883;
        RDMS_USERNAME = "****";
        RDMS_PASSWORD = "****";
    }else if("Beta" == mqtt_broker_){
        RDMS_HOST = "****";
        RDMS_PORT = 8883;
        RDMS_USERNAME = "****";
        RDMS_PASSWORD = "****";
    }else if("Prod" == mqtt_broker_){
        RDMS_HOST = "****";
        RDMS_PORT = 8883;
        RDMS_USERNAME = "****";
        RDMS_PASSWORD = "****";
    }else{
        RDMS_HOST = "****";
        RDMS_PORT = 1883;
        RDMS_USERNAME = "****";
        RDMS_PASSWORD = "****";
    }

    if( RdmsTopicBase::rdms_mqtt_ == nullptr){
        RdmsTopicBase::rdms_mqtt_ =  std::make_unique<Mqtt>(RDMS_HOST.c_str(), RDMS_PORT, 
                                RDMS_USERNAME.c_str(), RDMS_PASSWORD.c_str(), robotid_.c_str());
        rdms_mqtt_->setDebug(debug_);
        ROS_INFO("RDMS_HOST: %s", RDMS_HOST.c_str());
        ROS_INFO("RDMS_PORT: %d", RDMS_PORT);
        // ROS_INFO("RDMS_USERNAME: %s", RDMS_USERNAME.c_str());
        // ROS_INFO("RDMS_PASSWORD: %s", RDMS_PASSWORD.c_str());
    }
}

void RdmsTopicBase::rosParamInit(){
    int temp_site_id;
    //ros param
    ros_param_.param<bool>("debug", debug_, false);

    ros_param_.param<std::string>("/api_version", std_api_version_, "undefined");
    ros_param_.param<std::string>("/robot_id", robotid_, "undefined");
    ros_param_.param<std::string>("/rdms_service_name", service_name_, "undefined");
    ros_param_.param<std::string>("/site_id", siteid_, "undefined");

    ros_param_.param<std::string>("broker", mqtt_broker_, "undefined");

    if(siteid_ == "undefined"){
        ros_param_.param<int>("/site_id", temp_site_id, -1);
        siteid_ = (temp_site_id == -1? "undefined": std::to_string(temp_site_id));
    }

    //load mqtt topic
    topic_request_.standard_api_version = std_api_version_;
    topic_request_.service_name = service_name_;
    topic_request_.robot_id = robotid_;
    topic_request_.site_id = siteid_;

    topic_info_ = topic_request_;
    topic_response_ = topic_request_;

    topic_response_.message_type = "response";
    topic_request_.message_type = "request";
    topic_info_.message_type = "info";
}

int64_t RdmsTopicBase::getUnixTimeStamp(){
        std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
        std::chrono::system_clock::now().time_since_epoch()
    );
    return ms.count();
}

std::string RdmsTopicBase::generateUUID(){
    boost::uuids::uuid a_uuid = boost::uuids::random_generator()(); 
    return boost::uuids::to_string(a_uuid);
}


void RdmsTopicBase::publishTopic(const std::string& topic, const std::string& message, const int& qos){
    if(rdms_mqtt_ == nullptr) return;
    rdms_mqtt_->sendMessage(topic, message, qos);
}


void RdmsTopicBase::topicInit(const std::string& msg_category, const std::string& msg_name)
{
   topic_request_.message_category = (msg_category.empty() == false? msg_category: "undefined");
   topic_response_.message_category = topic_request_.message_category;
   topic_info_.message_category = topic_request_.message_category;

   topic_request_.message_name = (msg_name.empty() == false? msg_name: "undefined"); 
   topic_response_.message_name = topic_request_.message_name;
   topic_info_.message_name = topic_request_.message_name;
}

} //end namesapce zros