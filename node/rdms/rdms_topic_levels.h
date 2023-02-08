/**
 * @file rdms_topic_levels.h
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __RDMS_TOPIC_LIVELS_H__
#define __RDMS_TOPIC_LIVELS_H__

#include <string>
#include <ostream>
#include <map>
#include "../../lib/reflection/reflection.h"

using emptyData = std::map<std::string, std::string>;

template<typename T>
struct MqttInfoPayload{
    std::string clientId;
    int64_t timestamp;
    T data;
}; //end struct MqttInfoHeader

#define DEFINE_STRUCT_INFO_FIELD(DataType)\
        DEFINE_STRUCT_SCHEMA(MqttInfoPayload<DataType>,\
        DEFINE_STRUCT_FIELD(clientId, "clientId"), \
        DEFINE_STRUCT_FIELD(timestamp, "timestamp"),\
        DEFINE_STRUCT_FIELD(data, "data"));

template<>
struct MqttInfoPayload<emptyData>{
    std::string clientId;
    int64_t timestamp;
    emptyData data;
    
}; //end struct MqttInfoHeader


DEFINE_STRUCT_SCHEMA(MqttInfoPayload<emptyData>,\
        DEFINE_STRUCT_FIELD(clientId, "clientId"), \
        DEFINE_STRUCT_FIELD(timestamp, "timestamp"),\
        DEFINE_STRUCT_FIELD(data, "data"));

template<typename T>
struct MqttResponePayload{
    std::string correlationId;
    std::string clientId;
    int64_t timestamp;
    std::string result;
    std::string message;
    T data;
}; //end struct MqttRequestHeader

#define DEFINE_STRUCT_RESPONE_FIELD(DataType)\
        DEFINE_STRUCT_SCHEMA(MqttResponePayload<DataType>,\
        DEFINE_STRUCT_FIELD(correlationId, "correlationId"),\
        DEFINE_STRUCT_FIELD(clientId, "clientId"),\
        DEFINE_STRUCT_FIELD(timestamp, "timestamp"),\
        DEFINE_STRUCT_FIELD(result, "result"),\
        DEFINE_STRUCT_FIELD(message, "message"),\
        DEFINE_STRUCT_FIELD(data, "data"));

template<>
struct MqttResponePayload<emptyData>{
    std::string correlationId;
    std::string clientId;
    int64_t timestamp;
    std::string result;
    std::string message;
    emptyData data;
}; //end struct MqttRequestHeader


DEFINE_STRUCT_SCHEMA(MqttResponePayload<emptyData>,\
        DEFINE_STRUCT_FIELD(correlationId, "correlationId"),\
        DEFINE_STRUCT_FIELD(clientId, "clientId"),\
        DEFINE_STRUCT_FIELD(timestamp, "timestamp"),\
        DEFINE_STRUCT_FIELD(result, "result"),\
        DEFINE_STRUCT_FIELD(message, "message"),\
        DEFINE_STRUCT_FIELD(data, "data"));

template<typename T>
struct MqttRequestPayload{
    std::string correlationId;
    std::string clientId;
    int64_t timestamp;
    T data;
}; //end struct MqttResponeHeader

#define DEFINE_STRUCT_REQUEST_FIELD(DataType)\
        DEFINE_STRUCT_SCHEMA(MqttRequestPayload<DataType>,\
        DEFINE_STRUCT_FIELD(correlationId, "correlationId"),\
        DEFINE_STRUCT_FIELD(clientId, "clientId"),\
        DEFINE_STRUCT_FIELD(timestamp, "timestamp"),\
        DEFINE_STRUCT_FIELD(data, "data"));

template<>
struct MqttRequestPayload<emptyData>{
    std::string correlationId;
    std::string clientId;
    int64_t timestamp;
    emptyData data;
}; //end struct MqttResponeHeader

DEFINE_STRUCT_SCHEMA(MqttRequestPayload<emptyData>,\
                    DEFINE_STRUCT_FIELD(correlationId, "correlationId"),\
                    DEFINE_STRUCT_FIELD(clientId, "clientId"),\
                    DEFINE_STRUCT_FIELD(timestamp, "timestamp"),\
                    DEFINE_STRUCT_FIELD(data, "data"));

namespace zros{

struct RdmsTopicLevels{
    std::string service_name;
    std::string standard_api_version;
    std::string message_type; // message type[request/response/info]
    std::string site_id;
    std::string robot_id;
    std::string message_category; // message category [delivery/move/function]
    std::string message_name; // name of the detailed message

    std::string getTopicLevels() const {
        return (service_name+"/"+standard_api_version+"/"+message_type+"/"+\
                site_id+"/"+robot_id+"/"+message_category+"/"+message_name);
    }

    bool operator==(const RdmsTopicLevels& topic) const {
        if(topic.standard_api_version == this->standard_api_version&&
            topic.message_category == this->message_category &&
            topic.message_name == this->message_name &&
            topic.message_type == this->message_type &&
            topic.service_name == this->service_name &&
            topic.robot_id == this->robot_id &&
            topic.site_id == this->site_id){
                return true;
            }
        return false;
    }

    bool operator!=(const RdmsTopicLevels& topic) const {
        return !(topic == *this);
    }

}; //end struct RdmsTopicLevels

} //end namespace zros 


#endif /*__RDMS_TOPIC_LIVELS_H__*/