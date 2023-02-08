#ifndef __COMDER_TOPIC_BASE_H__
#define __COMDER_TOPIC_BASE_H__

#include <ros/ros.h>
#include <std_msgs/String.h>

#include <string>
#include <memory> //for smart pointer

#include <nlohmann/json.hpp>

#include "../../lib/mqtt/mqtt.h"
#include "comder_topic_livels.h"

namespace zros{
class ComderTopicBase{
public:
    ComderTopicBase();
    virtual ~ComderTopicBase();
    virtual void init() = 0;

private:
    ros::NodeHandle ros_nh_;
    ros::NodeHandle ros_param_;
    static std::unique_ptr<Mqtt> cmder_mqtt_;
    std::vector<std::string> sub_topic_vec_;
    std::string std_api_version_;
    std::string mqtt_broker_;
    std::string robotid_;
    std::string siteid_;
    std::string service_name_;
    bool debug_;

    ComderTopicLevels topic_request_;
    ComderTopicLevels topic_response_;
    ComderTopicLevels topic_info_;

    void siwtchBroker();
    void rosParamInit();
    ComderTopicBase(const ComderTopicBase&) = delete;
    ComderTopicBase(ComderTopicBase&&) = delete;
    ComderTopicBase& operator=(const ComderTopicBase&) = delete;
    ComderTopicBase& operator=(ComderTopicBase&&) = delete;

protected:
    void topicInit(const std::string& msg_name);
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
        if(cmder_mqtt_ == nullptr) return;
        sub_topic_vec_.push_back(topic);
        cmder_mqtt_->subscribeTopic(topic, std::forward<Function>(fcn), \
                                std::forward<Args>(args)...);
    }

    void unsubscribe(const std::string& topic){
        cmder_mqtt_->unsubscribe(NULL, topic.c_str());
    }

    void publishTopic(const std::string& topic, const std::string& message, const int& qos = 1);

    template<typename T>
    void publishTopic(const std::string& topic, const T& message, const int& qos = 1){
        publishTopic(topic, nlohmann::json(message).dump(), qos);
    }
};
} //end namespace zros

#endif /*__COMDER_TOPIC_BASE_H__*/