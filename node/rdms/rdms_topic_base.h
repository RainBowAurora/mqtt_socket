/**
 * @file rdms_topic_base.h
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __RDMS_TOPIC_BASE_H__
#define __RDMS_TOPIC_BASE_H__


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <memory>
#include <string>

#include <nlohmann/json.hpp>

#include "../../lib/mqtt/mqtt.h"
#include "../../lib/execute/execute_cmd.h"
#include "rdms_topic_levels.h"


namespace zros{

class RdmsTopicBase{
public:
    RdmsTopicBase();
    virtual ~RdmsTopicBase();
    virtual void init() = 0;

private:
    ros::NodeHandle ros_param_;
    ros::NodeHandle ros_nh_;
    std::string service_name_; //Name of service that specifies the standard API
    std::string std_api_version_; //Standard api version
    RdmsTopicLevels topic_request_;
    RdmsTopicLevels topic_response_;
    RdmsTopicLevels topic_info_;
    std::string siteid_; //ID of the site where the robot is registered
    std::string robotid_; //Id of the robot(MQTT client ID)
    std::string mqtt_broker_;
    static std::unique_ptr<Mqtt> rdms_mqtt_;
    bool debug_;
    std::vector<std::string> sub_topic_vec_;

    RdmsTopicBase(const RdmsTopicBase&) = delete;
    RdmsTopicBase(RdmsTopicBase&&) = delete;
    RdmsTopicBase& operator=(const RdmsTopicBase&) = delete;
    RdmsTopicBase& operator=(RdmsTopicBase&&) = delete;

    void rosParamInit();
    void rosTimeTaskInit();
    void switchBroker();
protected:
    void topicInit(const std::string& msg_category, const std::string& msg_name);
    int64_t getUnixTimeStamp();
    std::string generateUUID();
    std::string getReqTopicName() const { return topic_request_.getTopicLevels(); }
    std::string getResTopicName() const { return topic_response_.getTopicLevels(); }
    std::string getInfoTopicName() const { return topic_info_.getTopicLevels(); }
    std::string getRobotID() const { return  robotid_; }
    std::string getSiteID() const { return siteid_; }

    template<typename T>
    void parseMessage(const std::string& message, T& result){
        result = nlohmann::json::parse(message).get<T>();
    }

    template<class Function, class... Args>
    void subscribeTopic(const std::string& topic, Function&& fcn, Args&&... args){
        if(rdms_mqtt_ == nullptr) return;
        sub_topic_vec_.push_back(topic);
        rdms_mqtt_->subscribeTopic(topic, std::forward<Function>(fcn), \
                                std::forward<Args>(args)...);
    }

    void unsubscribe(const std::string& topic){
        rdms_mqtt_->unsubscribe(NULL, topic.c_str());
    }

    void publishTopic(const std::string& topic, const std::string& message, const int& qos = 1);

    template<typename T>
    void publishTopic(const std::string& topic, const T& message, const int& qos = 1){
        publishTopic(topic, nlohmann::json(message).dump(), qos);
    }

}; //end class RdmsTopicBase


} //end namespace zros




#endif /*__RDMS_TOPIC_BASE_H__*/