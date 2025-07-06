#pragma once

#include "TeamplayTypes.hpp"

namespace falcons
{

/**
 * ActionChoice handles the cycling through different action types.
 * Contains the tick counter logic for action decisions.
 */
class ActionChoice
{
public:
    ActionChoice();

    /**
     * Get the current action based on tick counter logic
     * @return Current action to execute
     */
    Action getCurrentAction();

    /**
     * Reset the action choice state
     */
    void reset();

private:
    void updateAction();

    int ticks_remaining_;
    Action current_action_;
};

} // namespace falcons
