#pragma once

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <string>
#include <memory>
#include <stdexcept>

namespace config {

class ParameterLoader
{
public:
    ParameterLoader(rclcpp::Node& node, const std::string& yaml_file)
    : node_(node)
    {
        try {
            root_ = YAML::LoadFile(yaml_file);
        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to load YAML config: " + std::string(e.what()));
        }
        declareRecursive(root_, "");
    }

    template<typename T>
    T get(const std::string& key) const
    {
        return node_.get_parameter(key).template get_value<T>();
    }

    bool has(const std::string& key) const
    {
        return node_.has_parameter(key);
    }

private:
    rclcpp::Node& node_;
    YAML::Node root_;

    void declareRecursive(const YAML::Node& node, const std::string& prefix)
    {
        for (const auto& it : node) {
            const std::string key = it.first.as<std::string>();
            const YAML::Node& val = it.second;
            const std::string full_key = prefix.empty() ? key : prefix + "." + key;
            if (isValueLeaf(val)) {
                declareLeaf(full_key, val);
            } else {
                declareRecursive(val, full_key);
            }
        }
    }

    // Accept either a scalar or a map with a "value" key as a leaf
    bool isValueLeaf(const YAML::Node& node)
    {
        // A scalar is always a leaf
        if (node.IsScalar()) return true;
        // A map with a "value" key is also a leaf
        if (node.IsMap() && node["value"] && node["value"].IsDefined()) return true;
        return false;
    }

    void declareLeaf(const std::string& key, const YAML::Node& node)
    {
        rcl_interfaces::msg::ParameterDescriptor desc;

        // If this is a map, check for description/min/max
        const YAML::Node* value_node_ptr = nullptr;
        if (node.IsMap()) {
            if (node["description"] && node["description"].IsDefined())
                desc.description = node["description"].as<std::string>();
            if (node["min"] && node["max"] && node["min"].IsDefined() && node["max"].IsDefined()) {
                const YAML::Node& val = (node["value"] && node["value"].IsDefined()) ? node["value"] : node;
                if (val.IsScalar()) {
                    const std::string val_str = val.Scalar();
                    if (val_str.find('.') != std::string::npos) {
                        rcl_interfaces::msg::FloatingPointRange range;
                        range.from_value = node["min"].as<double>();
                        range.to_value = node["max"].as<double>();
                        range.step = 0.0;
                        desc.floating_point_range.push_back(range);
                    } else {
                        rcl_interfaces::msg::IntegerRange range;
                        range.from_value = node["min"].as<int64_t>();
                        range.to_value = node["max"].as<int64_t>();
                        range.step = 1;
                        desc.integer_range.push_back(range);
                    }
                }
            }
        }

        // Robust: always bind value_node to a valid lvalue reference
        const YAML::Node& value_node = (node.IsMap() && node["value"] && node["value"].IsDefined()) ? node["value"] : node;

        if (value_node.IsScalar()) {
            try {
                double v = value_node.as<double>();
                node_.declare_parameter<double>(key, v, desc);
                return;
            } catch (...) {}

            try {
                int v = value_node.as<int>();
                node_.declare_parameter<int>(key, v, desc);
                return;
            } catch (...) {}

            try {
                bool v = value_node.as<bool>();
                node_.declare_parameter<bool>(key, v, desc);
                return;
            } catch (...) {}

            try {
                std::string v = value_node.as<std::string>();
                node_.declare_parameter<std::string>(key, v, desc);
                return;
            } catch (...) {}
        }

        throw std::runtime_error("Unsupported parameter type or format for key: " + key);
    }
};

} // namespace config

class ConfigurationROS {
public:
    ConfigurationROS(rclcpp::Node *node, const std::string &config_file_path)
    {
        loader_ = std::make_unique<config::ParameterLoader>(*node, config_file_path);
    }

    template<typename T>
    T get(const std::string& key) const
    {
        return loader_->get<T>(key);
    }

    bool has(const std::string& key) const
    {
        return loader_->has(key);
    }

    YAML::Node get_scope(const std::string &key)
    {
        // TODO
        return YAML::Node();
    }
private:
    std::unique_ptr<config::ParameterLoader> loader_;
};
