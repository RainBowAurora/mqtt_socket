#include "mqtt.h"
#include <algorithm>
#include <cstring>

/**
 * @brief Mosquitto API Ref: https://mosquitto.org/api/files/mosquitto-h.html
 * 
 */

namespace zros{

Mqtt::Mqtt(const char* host, int port, 
            const char* username, const char* pwd, const char *id): host_(host), 
                port_(port), username_(username), pwd_(pwd), mosqpp::mosquittopp(id)
{
    init();
}

Mqtt::~Mqtt()
{
    switch(disconnect()){
        case MOSQ_ERR_SUCCESS:
            std::cout << "[OK] Mqtt disconnect on success.\n"; break;
        case MOSQ_ERR_INVAL:
            std::cout << "[ERROR] Mqtt disconnect failed the input parameters were invalid.\n"; break;
        case MOSQ_ERR_NO_CONN:
            std::cout << "[ERROR] Mqtt disconnect failed the client isn’t connected to a broker.\n"; break;
        default:
            std::cout << "[ERROR] Mqtt disconnect unknow error.\n"; break;
    }

    switch(loop_stop()){  // Kill the thread
        case MOSQ_ERR_SUCCESS:
            std::cout << "[OK] Mqtt loop_stop on success.\n"; break;
        case MOSQ_ERR_INVAL:
            std::cout << "[ERROR] Mqtt loop_stop failed the input parameters were invalid.\n"; break;
        case MOSQ_ERR_NOT_SUPPORTED:
            std::cout << "[ERROR] Mqtt loop_stop failed thread support is not available.\n"; break;
        default:
            std::cout << "[ERROR] Mqtt loop_stop unknow error.\n"; break;
    }         

    mosqpp::lib_cleanup(); // Call to free resources associated with the library.
}

void Mqtt::sendMessage(const std::string& topic, const std::string& message, const int& qos, const bool& retain)
{
    if(topic.empty() || message.empty()) return; 

    if(deubg_){
        std::cout << "===\n";
        std::cout << "[<<] Mqtt - SendMessage msg->topic: "<< topic << "\tmsg->qos: "  << \
                        qos << "\tmsg->payload: " << message << std::endl;
    }

    switch(publish(NULL, topic.c_str(), strlen(message.c_str())+1, message.c_str(), qos, retain)){
        case MOSQ_ERR_SUCCESS:
            std::cout << "[OK] Mqtt publish " << topic << " on success.\n";break;
        case MOSQ_ERR_INVAL:
            std::cout << "[ERROR] Mqtt publish " << topic << " the input parameters were invalid.\n"; break;
        case MOSQ_ERR_NOMEM:
            std::cout << "[ERROR] Mqtt publish " << topic << " an out of memory condition occurred.\n"; break;
        case MOSQ_ERR_NO_CONN:
            std::cout << "[ERROR] Mqtt publish " << topic << " the client isn’t connected to a broker.\n"; break;
        case MOSQ_ERR_PROTOCOL:
            std::cout << "[ERROR] Mqtt publish " << topic << " there is a protocol error communicating with the broker.\n"; break;
        case MOSQ_ERR_PAYLOAD_SIZE:
            std::cout << "[ERROR] Mqtt publish " << topic << " payloadlen is too large.\n"; break;
        case MOSQ_ERR_MALFORMED_UTF8:
            std::cout << "[ERROR] Mqtt publish " << topic << " the topic is not valid UTF-8\n"; break;
        case MOSQ_ERR_QOS_NOT_SUPPORTED:
            std::cout << "[ERROR] Mqtt publish " << topic << " the QoS is greater than that supported by the broker.\n"; break;
        case MOSQ_ERR_OVERSIZE_PACKET:
            std::cout << "[ERROR] Mqtt publish " << topic << " the resulting packet would be larger than supported by the broker.\n"; break;
        default:
            std::cout << "[ERROR] Mqtt publish unknow error.\n"; break;
    }
}

void Mqtt::retisterTopic(const std::string& topic)
{
    if(topic.empty()) return;
    switch(subscribe(NULL, topic.c_str(), 1)){
        case MOSQ_ERR_SUCCESS:
            std::cout << "[OK] Mqtt subscribe " << topic <<  " on success.\n"; break;
        case MOSQ_ERR_INVAL:
            std::cout << "[ERROR] Mqtt subscribe " << topic << " the input parameters were invalid.\n"; break;
        case MOSQ_ERR_NOMEM:
            std::cout << "[ERROR] Mqtt subscribe " << topic << " an out of memory condition occurred.\n"; break;
        case MOSQ_ERR_NO_CONN:
            std::cout << "[ERROR] Mqtt subscribe " << topic << " the client isn’t connected to a broker.\n"; break;
        case MOSQ_ERR_MALFORMED_UTF8:
            std::cout << "[ERROR] Mqtt subscribe " << topic << " the topic is not valid UTF-8\n"; break;
        case MOSQ_ERR_OVERSIZE_PACKET:
            std::cout << "[ERROR] Mqtt subscribe " << topic << " he resulting packet would be larger than supported by the broker.\n"; break;
        default:
            std::cout << "[ERROR] Mqtt subscribe unknow error.\n"; break;
    }
}

void Mqtt::init()
{
    switch(mosqpp::lib_init()){
        case MOSQ_ERR_SUCCESS:
            std::cout << "[OK] mosquitto init.\n"; break;
        case MOSQ_ERR_UNKNOWN:
            std::cout << "[ERROR] couldn’t be initialized.\n"; break;
        default:
            std::cout << "[ERROR] unknow error.\n"; break;
    }

    switch(username_pw_set(username_.c_str(), pwd_.c_str())){
        case MOSQ_ERR_SUCCESS:
            std::cout << "[OK] mosquitto username and password set.\n"; break;
        case MOSQ_ERR_UNKNOWN:
            std::cout << "[ERROR] couldn’t set username and password.\n"; break;
        default:
            std::cout << "[ERROR] unknow error.\n"; break;
    }

    // switch(opts_int_set(MOSQ_OPT_TLS_USE_OS_CERTS, 1)){
    //     case MOSQ_ERR_SUCCESS:
    //         std::cout << "[OK] mosquitto use os certs. \n"; break;    
    //     default:
    //         std::cout << "[ERROR] set use os certs failed. \n"; break;
    // }

    switch(connect_async(host_.c_str(), port_, keeplive_)){
        case MOSQ_ERR_SUCCESS:
            std::cout << "[OK] Connect to an MQTT broker.\n"; break;
        case MOSQ_ERR_INVAL:
            std::cout << "[ERROR] input parameters were invalid.\n"; break;
        case MOSQ_ERR_ERRNO:
            std::cout << "[ERROR] system call returned an error..\n"; break;
        default:
            std::cout << connect_async(host_.c_str(), port_, keeplive_) << std::endl; break;
    }

    switch(loop_start()){
        case MOSQ_ERR_SUCCESS:
            std::cout << "[OK] Start thread managing connection publish/subscribe\n"; break;
        case MOSQ_ERR_INVAL:
            std::cout << "[ERROR] input parameters were invalid.\n"; break;
        case MOSQ_ERR_NOT_SUPPORTED:
            std::cout << "[ERROR] thread support is not available.\n";  break;
        default:
            std::cout << "[ERROR] unknow error.\n"; break;
    }

}

void Mqtt::on_connect(int rc)
{
    if ( rc == 0 ) {
        std::cout << "[>>] Mqtt - connected with server\n";
    } else {
        std::cout << "[>>] Mqtt - Impossible to connect with server(" << rc << ")\n";
    }
}

void Mqtt::on_disconnect(int rc)
{
    std::cout << ">> Mqtt - disconnection(" << rc << ")\n";
}

void Mqtt::on_publish(int mid)
{
    if(deubg_){
        std::cout << "[>>] Mqtt - Message (" << mid << ") succeed to be published \n";
    }
}

void Mqtt::on_message(const struct mosquitto_message * message)
{
    if(message->payload == NULL) return;

    if(deubg_){
        std::cout << "===\n";
        std::cout << ">> Mqtt ReceiveMessage msg->topic: "<< message->topic << "\tmsg->qos: "  << \
                        message->qos << "\tmsg->payload: " << (char *)message->payload << std::endl;
    }

    if(subscribe_.find(message->topic) != subscribe_.end()){
        std::cout << "[OK] Mqtt recdive " << message->topic << std::endl;
        subscribe_[message->topic]((char*)message->payload); 
    }else{
        std::cout << "[WARN] topic " << message->topic << " can not find the callback function\n";
    }
}

/**
 * @brief Set the subscribe callback.  This is called when the broker responds to a subscription request.
 * 
 */
void Mqtt::on_subscribe(int mid, int qos_count, const int *granted_qos)
{
    if(deubg_){
        std::cout << "[>>] Mqtt - Message (" << mid << ") succeed to be subcribe\n";
    }
}

void Mqtt::on_unsubscribe(int mid)
{
    if(deubg_){
        std::cout << "[>>] Mqtt - unsubscribe (" << mid << ") succeed to be unsubcribe\n";
    }
}

void Mqtt::on_log(int level, const char * str)
{
    if(deubg_){
        std::cout << "[>>] Mqtt - log level: " << level << "brief: " << str << std::endl;
    }
}


} // end namespace zros