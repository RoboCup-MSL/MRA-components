#ifndef FALCONS_ACTION_BASE_HPP
#define FALCONS_ACTION_BASE_HPP

#include <string>
#include <memory>
#include <type_traits>
#include <cstdint>
#include "mra_common_msgs/msg/action_type.hpp"
#include "mra_common_msgs/msg/action_result.hpp"
#include "mra_common_msgs/msg/world_state.hpp"

// Template base class for actions
template <typename InputMsg, typename ConfigMsg>
class ActionBase
{
public:
    using input_type = InputMsg;
    using config_type = ConfigMsg;

    ActionBase(const std::string& name, uint8_t type)
    : name_(name), type_(type), actionresult_(mra_common_msgs::msg::ActionResult::ACTIONRESULT_INVALID)
    {}

    virtual ~ActionBase() = default;

    // Called once at startup or config reload
    virtual void initialize(const config_type& config) { config_ = config; }

    // Called every tick with new input
    virtual void tick(const mra_common_msgs::msg::WorldState& world_state, const input_type& input) = 0;

    // Called at shutdown or when action is finished
    //virtual void finalize() {}

    // Getters
    std::string getName() const { return name_; }
    uint8_t getType() const { return type_; }
    uint8_t getActionResult() const { return actionresult_; }
    std::string getVerdict() const { return verdict_; }
    const config_type& getConfig() const { return config_; }

protected:
    std::string name_;
    uint8_t type_; // Use ActionType constants
    uint8_t actionresult_; // Use ActionResult constants
    std::string verdict_;
    config_type config_;
};

#endif // FALCONS_ACTION_BASE_HPP
