/**
 * @file cmder_topic_livels.h
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-06-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __CMDER_TOPIC_LIVELS_H__
#define __CMDER_TOPIC_LIVELS_H__

#include <string>
#include <ostream>
#include <map>
#include "../../lib/reflection/reflection.h"

template<typename T>
struct ComderReqPlayload {
    std::string correlationId;
    std::string clientId;
    int64_t timestamp;
    T data;
}; //end struct ComderReqPlayload
#define CMDER_DEFINE_STRUCT_REQ_FIELD(DataType)\
        DEFINE_STRUCT_SCHEMA(ComderReqPlayload<DataType>,\
        DEFINE_STRUCT_FIELD(correlationId, "correlationId"),\
        DEFINE_STRUCT_FIELD(clientId, "clientId"),\
        DEFINE_STRUCT_FIELD(timestamp, "timestamp"),\
        DEFINE_STRUCT_FIELD(data, "data"));

template<typename T>
struct ComderResPlayPoad{
    std::string correlationId;
    std::string clientId;
    int64_t timestamp;
    std::string result;
    std::string message;
    T data;
}; //end staruct ComderResPlayPoad
#define CMDER_DEFINE_STRUCT_RES_FIELD(DataType)\
        DEFINE_STRUCT_SCHEMA(ComderResPlayPoad<DataType>,\
        DEFINE_STRUCT_FIELD(correlationId, "correlationId"),\
        DEFINE_STRUCT_FIELD(clientId, "clientId"),\
        DEFINE_STRUCT_FIELD(timestamp, "timestamp"),\
        DEFINE_STRUCT_FIELD(message, "message"),\
        DEFINE_STRUCT_FIELD(data, "data"));


template<typename T>
struct ComderInfoPayload {
    std::string clientId;
    int64_t timestamp;
    T data;
}; //end struct ComderInfoPayload
#define CMDER_DEFINE_STRUCT_INFO_FIELD(DataType)\
        DEFINE_STRUCT_SCHEMA(ComderInfoPayload<DataType>,\
        DEFINE_STRUCT_FIELD(clientId, "clientId"),\
        DEFINE_STRUCT_FIELD(timestamp, "timestamp"),\
        DEFINE_STRUCT_FIELD(data, "data"));

namespace zros{

struct ComderTopicLevels{
    std::string service_name;
    std::string standard_api_version;
    std::string message_type;
    std::string site_id;
    std::string robot_id;
    std::string message_name;

    std::string getTopicLevels() const {
        return (service_name + "/" + standard_api_version + "/" + message_type + "/" +\
                site_id + "/" + robot_id + "/" + message_name);
    }

    bool operator==(const ComderTopicLevels& topic) const{
        if(topic.standard_api_version == this->standard_api_version &&\
            topic.service_name == this->service_name&&\
            topic.message_type == this->message_type&&\
            topic.site_id == this->site_id&&\
            topic.robot_id == this->site_id&&\
            topic.message_name == this->message_name){
                return true;
            }
        return false;
    }

    bool operator!=(const ComderTopicLevels& topic) const{
        return !(topic == *this);
    }

};//end struct ComderTopicLevels

} //end namespace zros


#endif /*__CMDER_TOPIC_LIVELS_H__*/