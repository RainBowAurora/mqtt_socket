/**
 * @file connection.hpp
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-12
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __ROBOT_CONNECTION_HPP__
#define __ROBOT_CONNECTION_HPP__

#include "../../rdms_topic_factory.h"
#include "../../rdms_topic_base.h"

struct LwtData{
    bool alive;
};//end struct LwtData

DEFINE_STRUCT_SCHEMA(LwtData, \
                    DEFINE_STRUCT_FIELD(alive, "alive"));

struct ConnectionData{
    std::string clientId;
    LwtData data;
};
DEFINE_STRUCT_SCHEMA(ConnectionData, \
                    DEFINE_STRUCT_FIELD(clientId, "clientId"),\
                    DEFINE_STRUCT_FIELD(data, "data"));

namespace zros{

class RobotConnection final: public RdmsTopicBase{
public:
    RobotConnection(): ros_nh_(){

    }
    
    void init() override{
        RdmsTopicBase::topicInit("robot", "connection");
        timer_ = ros_nh_.createTimer(ros::Duration(3), &RobotConnection::sendAlive, this, true, true);
    }

private:
    ros::NodeHandle ros_nh_;
    ros::Timer timer_;
    
    void sendAlive(const ros::TimerEvent& e){
        ConnectionData robot_connection_playload;
        robot_connection_playload.clientId = getRobotID();
        robot_connection_playload.data.alive = false;

        publishTopic(getInfoTopicName(), robot_connection_playload);
    }

}; //end class 

REGIST_MQTT_TOPIC(RobotConnection, RobotConnection);

}//end namespace zros

#endif /**/