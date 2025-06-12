#ifndef FALCONS_ACTION_BASE_HPP
#define FALCONS_ACTION_BASE_HPP

#include <string>

class ActionBase
{
public:
    virtual ~ActionBase() = default;

    virtual void initialize() = 0;
    virtual void execute() = 0;
    virtual void finalize() = 0;

    virtual std::string getName() const = 0;
};

#endif // FALCONS_ACTION_BASE_HPP
