#ifndef __FACTORY_BASE_H__
#define __FACTORY_BASE_H__

#include <map>
#include <string>
#include <vector>
#include <memory>
#include <functional>
#include <set>

namespace zros{

template<typename ObjectType>
struct FactoryBase{  
using FuncPtrType = std::function<ObjectType* ()>;
using SharePtrType = std::shared_ptr<ObjectType>;
using UniquePtrType = std::unique_ptr<ObjectType>;
using objectMapType = std::map<std::string, FuncPtrType>;

ObjectType* ProducePtr(const std::string& key){
    if(object_map.find(key) == object_map.end()){
        return nullptr;
    }
        return object_map[key]();
}

SharePtrType ProduceSharedPtr(const std::string& key){
    return SharePtrType(ProducePtr(key));
}

UniquePtrType ProduceUniquePtr(const std::string& key){
    return UniquePtrType(ProducePtr(key));
}

size_t Size() const {
    return object_map.size();
}

bool Empyt() const {
    return object_map.empty();
}

std::vector<std::string> GetAllName() const {
    std::vector<std::string> result = {};
    for(auto element : object_map){
        result.push_back(element.first);
    }
    return result;
}

objectMapType object_map;

}; //end struct FactoryBase

} //end namespace zros

#endif /*__FACTORY_BASE_H__*/