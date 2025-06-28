#pragma once

#include <string>
#include <sstream>
#include <rosidl_runtime_cpp/traits.hpp>
#include <yaml-cpp/yaml.h>
#include <nlohmann/json.hpp>

// This json converter is based on https://github.com/mircodz/tojson (MIT license)
// Recursively convert a YAML::Node to nlohmann::json
namespace tojson {
namespace detail {

inline nlohmann::json parse_scalar(const YAML::Node &node) {
    int i;
    double d;
    bool b;
    std::string s;

    if (YAML::convert<int>::decode(node, i)) return i;
    if (YAML::convert<double>::decode(node, d)) return d;
    if (YAML::convert<bool>::decode(node, b)) return b;
    if (YAML::convert<std::string>::decode(node, s)) return s;

    return nullptr;
}

inline nlohmann::json yaml2json(const YAML::Node &root) {
    nlohmann::json j{};

    switch (root.Type()) {
    case YAML::NodeType::Null: break;
    case YAML::NodeType::Scalar: return parse_scalar(root);
    case YAML::NodeType::Sequence:
        for (auto &&node : root)
            j.emplace_back(yaml2json(node));
        break;
    case YAML::NodeType::Map:
        for (auto &&it : root)
            j[it.first.as<std::string>()] = yaml2json(it.second);
        break;
    default: break;
    }
    return j;
}

} // namespace detail
} // namespace tojson

inline std::string yaml_to_json(const YAML::Node &node) {
    return tojson::detail::yaml2json(node).dump();
}

template<typename MsgT>
inline std::string rosmsg_to_json(const MsgT &msg)
{
    static_assert(rosidl_generator_traits::is_message<MsgT>::value, "rosmsg_to_json requires a ROS 2 message type");
    std::string ys = to_yaml(msg);
    YAML::Node y = YAML::Load(ys);
    return yaml_to_json(y);
}
