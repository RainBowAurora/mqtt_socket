/**
 * @file mqtt.h
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-05-04
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __MQTT_H__
#define __MQTT_H__
#include <iostream>
#include <string>
#include <vector>
#include <map>
#include <functional>
#include <mosquitto.h>
#include <mosquittopp.h>

namespace zros{

class Mqtt final: public mosqpp::mosquittopp{
public:
    using ReceiveType = std::map<std::string, std::string>;
    using ReceviecallbackType = std::function<void(std::string)>;
    explicit Mqtt(const char* host, int port, 
                    const char* username, const char* pwd, const char *id=NULL);

    Mqtt() = default;
    ~Mqtt();
    void init();

    void setDebug(const bool flag) { deubg_ = flag; }
    std::string getHost() const { return host_; }
    int getPort() const { return  port_; }
    // std::string getUserName() const { return username_; }
    // std::string getPassword() const { return pwd_; }

    void sendMessage(const std::string& topic, const std::string& message, const int& qos = 1, const bool& retain = false);

    template<class Function, class... Args>
    void subscribeTopic(const std::string& topic, Function&& fcn, Args&&... args){
        subscribe_[topic] = std::bind(std::forward<Function>(fcn),std::forward<Args>(args)..., std::placeholders::_1);
        retisterTopic(topic);
    }

    void on_connect(int /*rc*/) override;
    // void on_connect_with_flags(int rc, int flags) override ;
    void on_disconnect(int /*rc*/) override;
    void on_publish(int /*mid*/) override;
    void on_message(const struct mosquitto_message * /*message*/) override;
    void on_subscribe(int /*mid*/, int /*qos_count*/, const int * /*granted_qos*/) override;
    void on_unsubscribe(int /*mid*/) override;
    void on_log(int /*level*/, const char * /*str*/) override;
    // void on_error();


private:
    std::string host_ = "undefined";
    std::string username_ = "undefined";
    std::string pwd_ = "undefined";
    int port_ = 1883;
    bool deubg_ = false;
    const int keeplive_ = 60;
    std::map<std::string, ReceviecallbackType> subscribe_;
    ReceiveType receive_data_;
    void retisterTopic(const std::string& topic);

}; //end namespace Mqtt


} // end namespace zros

#endif /*__MQTT_H__*/