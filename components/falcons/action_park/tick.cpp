// this file was produced by MRA-codegen.py from template_tick.cpp
// with the intent of letting user add the implementation here

// generated component header:
#include "FalconsActionPark.hpp"

using namespace MRA;

// custom includes, if any
#include "geometry.hpp"
#include <cmath>


void checkParams(FalconsActionPark::ParamsType const &params);
void checkRobotInTTA(Geometry::Point const &robotPos, FalconsActionPark::ParamsType const &params, bool &xInTTA, bool &yInTTA);
int getRelativeIndex(Datatypes::WorldState const &ws);
Geometry::Point calculateResultPos(int rId, FalconsActionPark::ParamsType const &params);
float calculateMinObjectDistance(float posX, float posY, Datatypes::WorldState const &ws);


int FalconsActionPark::FalconsActionPark::tick
(
    google::protobuf::Timestamp timestamp,   // absolute timestamp
    InputType  const           &input,       // input data, type generated from Input.proto
    ParamsType const           &params,      // configuration parameters, type generated from Params.proto
    StateType                  &state,       // state data, type generated from State.proto
    OutputType                 &output,      // output data, type generated from Output.proto
    DiagnosticsType            &diagnostics  // diagnostics data, type generated from Diagnostics.proto
)
{
    int error_value = 0;
    MRA_LOG_TICK();

    // user implementation goes here

    checkParams(params);

    // get robot position
    Geometry::Point robotPos;
    robotPos.x = input.worldstate().robot().position().x();
    robotPos.y = input.worldstate().robot().position().y();

    // check for success
    bool xCloseToCenter = fabs(robotPos.x - params.tta().center().x()) < params.tolerancex();
    bool xInTTA = false;
    bool yInTTA = false;
    checkRobotInTTA(robotPos, params, xInTTA, yInTTA);
    if (xCloseToCenter && yInTTA)
    {
        output.set_actionresult(MRA::Datatypes::ActionResult::PASSED);
        return error_value;
    }

    // find relative index compare to teammates
    int rId = getRelativeIndex(input.worldstate());

    // calculate target positions
    Geometry::Point targetPos = calculateResultPos(rId, params);
    Geometry::Point preTargetPos = targetPos;
    preTargetPos.x += params.pretargetxoffset() * (targetPos.x > 0 ? -1 : 1);

    // check if moving to pretarget or target
    float distanceToTarget = (targetPos - robotPos).size();
    if (distanceToTarget > params.pretargetdistance())
    {
        // calculate target angle
        float sign = (preTargetPos.x > 0) ? 1.0 : -1.0;
        output.mutable_motiontarget()->mutable_position()->set_rz(sign * 0.5 * M_PI);

        // set target
        output.mutable_motiontarget()->mutable_position()->set_x(preTargetPos.x);
        output.mutable_motiontarget()->mutable_position()->set_y(preTargetPos.y);
        output.set_actionresult(MRA::Datatypes::ActionResult::RUNNING);
    }
    else
    {
        // check if target is blocked
        float minDist = calculateMinObjectDistance(targetPos.x, targetPos.y, input.worldstate());
        bool targetBlocked = minDist < (params.robotradius() + params.robotclearance());

        // set target or determine action failure
        if (targetBlocked) // TODO: keep trying a bit more?
        {
            output.set_actionresult(MRA::Datatypes::ActionResult::FAILED);
        }
        else
        {
            // calculate target angle
            float sign = (targetPos.x > 0) ? 1.0 : -1.0;
            output.mutable_motiontarget()->mutable_position()->set_rz(sign * 0.5 * M_PI);

            // set target
            output.mutable_motiontarget()->mutable_position()->set_x(targetPos.x);
            output.mutable_motiontarget()->mutable_position()->set_y(targetPos.y);
            output.set_actionresult(MRA::Datatypes::ActionResult::RUNNING);
        }
    }

    return error_value;
}

void checkParams(FalconsActionPark::ParamsType const &params)
{
    MRA_TRACE_FUNCTION();
    float myEpsilon = 0.01;
    if (params.tta().size().x() <= myEpsilon)
    {
        throw std::runtime_error("invalid configuration, tta.size.x must be a nonzero value");
    }
    if (params.tta().size().y() <= myEpsilon)
    {
        throw std::runtime_error("invalid configuration, tta.size.y must be a nonzero value");
    }
    if (params.robotradius() <= myEpsilon)
    {
        throw std::runtime_error("invalid configuration, robotRadius must be a nonzero value");
    }
    if (params.tolerancex() <= myEpsilon)
    {
        throw std::runtime_error("invalid configuration, toleranceX must be a nonzero value");
    }
    // robotClearance is optional
}

void checkRobotInTTA(Geometry::Point const &robotPos, FalconsActionPark::ParamsType const &params, bool &xInTTA, bool &yInTTA)
{
    MRA_TRACE_FUNCTION();
    float ttaMinX = params.tta().center().x() - 0.5 * params.tta().size().x();
    float ttaMaxX = params.tta().center().x() + 0.5 * params.tta().size().x();
    float ttaMinY = params.tta().center().y() - 0.5 * params.tta().size().y();
    float ttaMaxY = params.tta().center().y() + 0.5 * params.tta().size().y();
    xInTTA = (ttaMinX <= robotPos.x) && (robotPos.x <= ttaMaxX);
    yInTTA = (ttaMinY <= robotPos.y) && (robotPos.y <= ttaMaxY);
    MRA_TRACE_FUNCTION_OUTPUTS(xInTTA, yInTTA);
}

int getRelativeIndex(Datatypes::WorldState const &ws)
{
    MRA_TRACE_FUNCTION();
    int own_id = ws.robot().id();
    const auto &teammates = ws.teammates();
    int relative_index = 0;
    for (const auto &teammate : teammates)
    {
        if (teammate.id() < own_id)
        {
            ++relative_index;
        }
    }
    MRA_TRACE_FUNCTION_OUTPUTS(relative_index);
    return relative_index;
}

Geometry::Point calculateResultPos(int rId, FalconsActionPark::ParamsType const &params)
{
    MRA_TRACE_FUNCTION();
    Geometry::Point result;
    float dy = ((rId + 1) / 2) * (rId & 1 ? 2 : -2) * (params.robotradius() + params.robotclearance());
    result.x = params.tta().center().x();
    result.y = params.tta().center().y() + dy;
    MRA_TRACE_FUNCTION_OUTPUTS(dy, result);
    return result;
}

float calculateMinObjectDistance(float posX, float posY, Datatypes::WorldState const &ws)
{
    MRA_TRACE_FUNCTION();
    float min_distance = std::numeric_limits<float>::max();
    auto calculateDistance = [&](float x, float y) {
        return std::sqrt(std::pow(x - posX, 2) + std::pow(y - posY, 2));
    };
    for (const auto &teammate : ws.teammates())
    {
        float distance = calculateDistance(teammate.position().x(), teammate.position().y());
        if (distance < min_distance)
        {
            min_distance = distance;
        }
    }
    for (const auto &opponent : ws.opponents())
    {
        float distance = calculateDistance(opponent.position().x(), opponent.position().y());
        if (distance < min_distance)
        {
            min_distance = distance;
        }
    }
    for (const auto &obstacle : ws.obstacles())
    {
        float distance = calculateDistance(obstacle.position().x(), obstacle.position().y());
        if (distance < min_distance)
        {
            min_distance = distance;
        }
    }
    MRA_TRACE_FUNCTION_OUTPUTS(min_distance);
    return min_distance;
}
