#pragma once

#include <string>
#include <memory>
#include <cstdint>
#include <nlohmann/json.hpp>
#include "Types.hpp"
#include "Configuration.hpp"

namespace falcons::action_planning
{

class ActionBase
{
public:
    ActionBase(const std::string& name, uint8_t type)
    : name_(name), type_(type)
    {}

    virtual ~ActionBase() = default;

    // Called once at action startup, based on yaml / ROS params
    virtual void initialize(const types::Settings& config) { config_ = config; }

    // Called every tick, to apply actionparams from input as overrule
    types::Settings mergeConfig(std::string actionparams = "")
    {
        types::Settings tick_config = config_;
        /*
        if (!actionparams.empty()) {
            try {
                nlohmann::json overrule_json = nlohmann::json::parse(actionparams);
                for (auto it = overrule_json.begin(); it != overrule_json.end(); ++it) {
                    tick_config[it.key()] = it.value();
                }
            } catch (const nlohmann::json::parse_error& e) {
                // Handle JSON parsing error, e.g., log it
            }
        }
        */
        return tick_config;
    }

    // Called every tick
    virtual void tick(
        const types::WorldState& world_state,
        const types::Settings& settings, // potentially overruled by tick input.actionparams
        types::ActionResult& action_result,
        types::Targets& targets
    ) = 0;

    virtual void finalize() {} // TODO: dump action data to file

    std::string getName() const { return name_; }

protected:
    std::string name_;
    uint8_t type_;
    types::Settings config_;
};

} // namespace falcons::action_planning
