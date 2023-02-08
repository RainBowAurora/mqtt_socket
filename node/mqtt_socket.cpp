#include "mqtt_socket.h"

namespace zros{
MqttSocket::MqttSocket(): ros_nh_(), ros_param_("~")
{
    rdmsInit();
    comderInit();
    timer_ = ros_nh_.createTimer(ros::Duration(3), 
                                &MqttSocket::uploadParam, this);
}

MqttSocket::~MqttSocket()
{
    
}

void MqttSocket::uploadParam(const ros::TimerEvent& e){
    std::string site_id;
    int temp_site_id;
    ros_param_.param<std::string>("/site_id", site_id, "undefined");
    if(site_id == "undefined"){
        ros_param_.param<int>("/site_id", temp_site_id, -1);
        site_id = (temp_site_id == -1? "undefined": std::to_string(temp_site_id));
    }
    static std::string last_site_id = site_id;
    if(last_site_id != site_id){
        rdmsInit();
    }
    last_site_id = site_id;
}


void MqttSocket::comderInit(){
    mqtt_comder_vec_.clear();
    for(auto element: CmderFactory::Instance().GetAllName()){
        mqtt_comder_vec_.push_back( CmderFactory::Instance().ProduceSharedPtr(element) );
    }

    for(auto element: mqtt_comder_vec_){
        element->init();
    }
}

void MqttSocket::rdmsInit()
{
    mqtt_rdms_vec_.clear();
    for(auto element: TopicFactory::Instance().GetAllName()){
        mqtt_rdms_vec_.push_back( TopicFactory::Instance().ProduceSharedPtr(element) );
    }

    for(auto element: mqtt_rdms_vec_){
        element->init();
    }
}

void MqttSocket::run()
{
    while(ros::ok()){
        ros::spinOnce();
        ros::Duration(0.5).sleep(); //sleep
    }
}

}// end namespace zros