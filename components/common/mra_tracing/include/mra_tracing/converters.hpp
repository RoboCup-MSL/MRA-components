#pragma once

#include <string>
#include <sstream>
#include <rosidl_runtime_cpp/traits.hpp>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>

inline std::string yaml_to_json(const YAML::Node &node) {
    return YAML::Dump(node); // TODO this is not JSON! but the json parser fails ...
    //nlohmann::json j = nlohmann::json::parse(YAML::Dump(node));
    //return j.dump();
}

template<typename MsgT>
inline std::string rosmsg_to_json(const MsgT &msg)
{
    static_assert(rosidl_generator_traits::is_message<MsgT>::value, "rosmsg_to_json requires a ROS 2 message type");
    std::ostringstream oss;
    // Use ADL to find the generated to_yaml function in the message's namespace
    std::string ys = to_yaml(msg);
    YAML::Node y = YAML::Load(ys);
    return yaml_to_json(y);
}
