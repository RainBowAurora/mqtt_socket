#ifndef __REFLECTION_H__
#define __REFLECTION_H__

#include "static_reflection.h"
#include "optional_json.h"

namespace {

template <typename T>
struct is_optional : std::false_type {};

template <typename T>
struct is_optional<std::unique_ptr<T>> : std::true_type {};

template <typename T>
constexpr bool is_optional_v = is_optional<std::decay_t<T>>::value;

template <typename T>
constexpr bool has_schema = std::tuple_size<decltype(StructSchema<T>())>::value;

}  // namespace


namespace nlohmann {

template <typename T>
struct adl_serializer<T, std::enable_if_t<::has_schema<T>>> {
  template <typename BasicJsonType>
  static void to_json(BasicJsonType& j, const T& value) {
    ForEachField(value, [&j](auto&& field, auto&& name) { j[name] = field; });
  }

  template <typename BasicJsonType>
  static void from_json(const BasicJsonType& j, T& value) {
    ForEachField(value, [&j](auto&& field, auto&& name) {
      // ignore missing field of optional
      if (::is_optional_v<decltype(field)> && j.find(name) == j.end())
        return;

      j.at(name).get_to(field);
    });
  }
};

} //end namespace nlohmann

#endif /*__REFLECTION_H__*/