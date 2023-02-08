#include "comder_topic_base.h"
#include <chrono>
#include <string>
#include <boost/uuid/uuid.hpp>
#include <boost/uuid/uuid_io.hpp>
#include <boost/uuid/uuid_generators.hpp>

namespace zros{
std::unique_ptr<Mqtt> ComderTopicBase::cmder_mqtt_ = nullptr;
    
ComderTopicBase::ComderTopicBase(): ros_nh_(), ros_param_("~"){
    rosParamInit();
    siwtchBroker();
}

ComderTopicBase::~ComderTopicBase(){
    for(auto element: sub_topic_vec_){
        unsubscribe(element);
    }
}

void ComderTopicBase::siwtchBroker(){
    int   CMDER_PORT = 0;
    std::string CMDER_HOST = "undefined";
    std::string CMDER_USERNAME = "undefined";
    std::string CMDER_PASSWORD = "undefined";

    if("Dev" == mqtt_broker_){
        CMDER_HOST = "****";
        CMDER_PORT = 8883;
        CMDER_USERNAME = "****";
        CMDER_PASSWORD = "****";
    }else if("Beta" == mqtt_broker_){
        CMDER_HOST = "****";
        CMDER_PORT = 8883;
        CMDER_USERNAME = "****";
        CMDER_PASSWORD = "****";
    }else if("Prod" == mqtt_broker_){
        CMDER_HOST = "****";
        CMDER_PORT = 8883;
        CMDER_USERNAME = "****";
        CMDER_PASSWORD = "****";
    }

    if(ComderTopicBase::cmder_mqtt_ == nullptr){
        ComderTopicBase::cmder_mqtt_ = std::make_unique<Mqtt>(CMDER_HOST.c_str(), CMDER_PORT, 
                                CMDER_USERNAME.c_str(), CMDER_PASSWORD.c_str(), robotid_.c_str());
        cmder_mqtt_->setDebug(debug_);     
        ROS_INFO("CMDER_HOST: %s", CMDER_HOST.c_str());
        ROS_INFO("CMDER_PORT: %d", CMDER_PORT);   
        // ROS_INFO("CMDER_USERNAME: %s", CMDER_USERNAME.c_str());
        // ROS_INFO("CMDER_PASSWORD: %s", CMDER_PASSWORD.c_str());                        
    }
}

void ComderTopicBase::rosParamInit(){
    int temp_site_id;
    ros_param_.param<bool>("debug", debug_, false);
    ros_param_.param<std::string>("/api_version", std_api_version_, "undefined");
    ros_param_.param<std::string>("/robot_id", robotid_, "undefined");
    ros_param_.param<std::string>("/cmder_service_name", service_name_, "undefined");
    ros_param_.param<std::string>("/comder_site_id", siteid_, "undefined");
    ros_param_.param<std::string>("broker", mqtt_broker_, "undefined");
    if(siteid_ == "undefined"){
        ros_param_.param<int>("/comder_site_id", temp_site_id, -1);
        siteid_ = (temp_site_id == -1? "undefined": std::to_string(temp_site_id));
    }

    topic_request_.standard_api_version =  std_api_version_;
    topic_request_.service_name =  service_name_;
    topic_request_.robot_id = "zhen-" + robotid_; 
    topic_request_.site_id = siteid_;

    topic_info_ = topic_request_;
    topic_response_ = topic_request_;

    topic_request_.message_type = "request";
    topic_response_.message_type = "response";
    topic_info_.message_type = "info";
}

int64_t ComderTopicBase::getUnixTimeStamp(){
        std::chrono::milliseconds ms = std::chrono::duration_cast< std::chrono::milliseconds >(
        std::chrono::system_clock::now().time_since_epoch()
    );
    return ms.count();
}

std::string ComderTopicBase::generateUUID(){
    boost::uuids::uuid a_uuid = boost::uuids::random_generator()(); 
    return boost::uuids::to_string(a_uuid);
}


void ComderTopicBase::publishTopic(const std::string& topic, const std::string& message, const int& qos){
    if(cmder_mqtt_ == nullptr) return;
    cmder_mqtt_->sendMessage(topic, message, qos);
}

void ComderTopicBase::topicInit(const std::string& msg_name)
{
   topic_request_.message_name = (msg_name.empty() == false? msg_name: "undefined"); 
   topic_response_.message_name = topic_request_.message_name;
   topic_info_.message_name = topic_request_.message_name;
}

} //end namespace zros
