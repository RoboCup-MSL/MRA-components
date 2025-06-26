#pragma once

#include <rclcpp/rclcpp.hpp>
#include <yaml-cpp/yaml.h>
#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/floating_point_range.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include <string>
#include <memory>
#include <stdexcept>
#include "mra_tracing/tracing.hpp"

namespace config {

class ParameterLoader
{
public:
    ParameterLoader(rclcpp::Node& node, const std::string& yaml_file)
    : node_(node)
    {
        TRACE_FUNCTION_INPUTS(yaml_file);
        try {
            root_ = YAML::LoadFile(yaml_file);
        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to load YAML config: " + std::string(e.what()));
        }
        declareRecursive(root_, "");
        TRACE_FUNCTION_OUTPUTS(root_);
    }

    template<typename T>
    T get(const std::string& key) const
    {
        TRACE_FUNCTION_INPUTS(key);
        T result = node_.get_parameter(key).template get_value<T>();
        TRACE_FUNCTION_OUTPUTS(key);
        return result;
    }

    bool has(const std::string& key) const
    {
        return node_.has_parameter(key);
    }

    // Return all declared parameter names
    const std::vector<std::string>& getDeclaredParameters() const
    {
        return declared_parameters_;
    }

    // Expose the YAML root node
    const YAML::Node& getYamlRoot() const
    {
        return root_;
    }

private:
    rclcpp::Node& node_;
    YAML::Node root_;
    std::vector<std::string> declared_parameters_;

    void declareRecursive(const YAML::Node& node, const std::string& prefix)
    {
        TRACE_FUNCTION_INPUTS(node, prefix);
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
        TRACE_FUNCTION_INPUTS(node);
        bool result = false;
        // A scalar is always a leaf
        if (node.IsScalar()) result = true;
        // A map with a "value" key is also a leaf
        if (node.IsMap() && node["value"] && node["value"].IsDefined()) result = true;
        TRACE_FUNCTION_OUTPUTS(result);
        return result;
    }

    void declareLeaf(const std::string& key, const YAML::Node& node)
    {
        TRACE_FUNCTION_INPUTS(key, node);
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
                declared_parameters_.push_back(key);
                return;
            } catch (...) {}

            try {
                int v = value_node.as<int>();
                node_.declare_parameter<int>(key, v, desc);
                declared_parameters_.push_back(key);
                return;
            } catch (...) {}

            try {
                bool v = value_node.as<bool>();
                node_.declare_parameter<bool>(key, v, desc);
                declared_parameters_.push_back(key);
                return;
            } catch (...) {}

            try {
                std::string v = value_node.as<std::string>();
                node_.declare_parameter<std::string>(key, v, desc);
                declared_parameters_.push_back(key);
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
        TRACE_FUNCTION();
        loader_ = std::make_unique<config::ParameterLoader>(*node, config_file_path);
    }

    template<typename T>
    T get(const std::string& key) const
    {
        TRACE_FUNCTION_INPUTS(key);
        T result;
        try {
            result = loader_->get<T>(key);
        } catch (const std::exception& e) {
            throw std::runtime_error("Failed to get parameter '" + key + "': " + e.what());
        }
        TRACE_FUNCTION_OUTPUTS(result);
        return result;
    }

    bool has(const std::string& key) const
    {
        return loader_->has(key);
    }

    YAML::Node get_scope(const std::string &key)
    {
        TRACE_FUNCTION_INPUTS(key);
        // Hard check: key must be a node in the YAML tree, to prevent typos in yaml file etc
        const YAML::Node& yaml_root = loader_->getYamlRoot();
        if (!yaml_root[key]) {
            throw std::runtime_error("Configuration key '" + key + "' does not exist in YAML root");
        }
        // Build a YAML node with only parameter values for the given key prefix
        YAML::Node result;
        std::string prefix = key.empty() ? "" : key + ".";
        for (const auto& param : loader_->getDeclaredParameters()) {
            if (param.rfind(prefix, 0) == 0) { // starts with prefix
                std::string subkey = param.substr(prefix.size());
                if (subkey.find('.') == std::string::npos) {
                    // leaf parameter
                    try {
                        if (loader_->has(param)) {
                            try {
                                result[subkey] = loader_->get<double>(param);
                                continue;
                            } catch (...) {}
                            try {
                                result[subkey] = loader_->get<int>(param);
                                continue;
                            } catch (...) {}
                            try {
                                result[subkey] = loader_->get<bool>(param);
                                continue;
                            } catch (...) {}
                            try {
                                result[subkey] = loader_->get<std::string>(param);
                                continue;
                            } catch (...) {}
                        }
                    } catch (...) {}
                }
            }
        }
        TRACE_FUNCTION_OUTPUTS(result);
        return result;
    }
private:
    std::unique_ptr<config::ParameterLoader> loader_;
};
