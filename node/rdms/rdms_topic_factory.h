#ifndef __RDMS_TOPIC_FACTORY_H__
#define __RDMS_TOPIC_FACTORY_H__

#include "rdms_topic_base.h"
#include "../../lib/factroy/factory_base.hpp"

namespace zros{

struct TopicFactory final: public FactoryBase<RdmsTopicBase>{  

static TopicFactory& Instance(){
    static TopicFactory instance;
    return instance;
}

template<typename T>
struct Register{
    template<typename... Args>
    Register(const std::string& key, Args&&... args){
        TopicFactory::Instance().object_map.emplace(key, [&]{ return new T(std::forward<T>(args)...); } );
    }
};

private:
    TopicFactory() = default;
    TopicFactory(const TopicFactory& ) = delete;
    TopicFactory(TopicFactory&& ) = delete;
    TopicFactory& operator=(const TopicFactory& ) = delete;
    TopicFactory& operator=(TopicFactory&& ) = delete;
};

} //end namespace zros

#define REGIST_MQTT_TOPIC_VNAME(T) reg_##T##_
#define REGIST_MQTT_TOPIC(T, key, ...) \
        static const zros::TopicFactory::Register<T> REGIST_MQTT_TOPIC_VNAME(T) (#key, ##__VA_ARGS__)


#endif /*__RDMS_TOPIC_FACTORY_H__*/