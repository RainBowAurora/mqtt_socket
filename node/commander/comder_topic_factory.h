/**
 * @file cmder_topic_factory.h
 * @author XiaoYuMa (xiaoyu.ma@zhenrobot.com)
 * @brief 
 * @version 0.1
 * @date 2022-06-29
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __COMDER_TOPIC_FACTORY_H__
#define __COMDER_TOPIC_FACTORY_H__

#include "comder_topic_base.h"
#include "../../lib/factroy/factory_base.hpp"

namespace zros{

struct CmderFactory final: public FactoryBase<ComderTopicBase>{
    static CmderFactory& Instance(){
        static CmderFactory instance;
        return instance;
    }

template<typename T>
struct Register{
    template<typename... Args>
    Register(const std::string& key, Args&&... args){
        CmderFactory::Instance().object_map.emplace(key, [&]{ return new T(std::forward<T>(args)...); } );
    }
};

private:
    CmderFactory() = default;
    CmderFactory(const CmderFactory&) = delete;
    CmderFactory(CmderFactory&&) = delete;
    CmderFactory& operator=(const CmderFactory&) = delete;
    CmderFactory& operator=(CmderFactory&&) = delete;
};//end struct CmderFactory

#define REGIST_COMDER_TOPIC_VNAME(T) reg_##T##_
#define REGIST_COMDER_TOPIC(T, key, ...) \
        static const zros::CmderFactory::Register<T> REGIST_COMDER_TOPIC_VNAME(T) (#key, ##__VA_ARGS__)

}//end namespace zros

#endif /*__COMDER_TOPIC_FACTORY_H__*/