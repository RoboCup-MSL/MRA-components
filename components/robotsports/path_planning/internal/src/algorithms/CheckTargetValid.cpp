/*
 * CheckTargetValid.cpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */

#include <algorithm>
 #include "PathPlanningAlgorithms.hpp"
#include "logging.hpp"
#include "cEnvironmentField.hpp"
// #include "cDiagnostics.hpp"


// determine what to do
void handleViolation(PathPlanningData &data, BoundaryOptionEnum mode)
{
    switch (mode)
    {
        case BoundaryOptionEnum::STOP_AND_PASS:
            data.done = true;
            data.stop = true;
            data.resultStatus = MRA::Datatypes::ActionResult::PASSED;
            break;
        case BoundaryOptionEnum::STOP_AND_FAIL:
            data.done = true;
            data.stop = true;
            data.resultStatus = MRA::Datatypes::ActionResult::FAILED;
            break;
        case BoundaryOptionEnum::CLIP:
            ; // falling through, do nothing (not in this function anyway)
        case BoundaryOptionEnum::ALLOW:
            ; // falling through, do nothing
        default:
            ; // do nothing
    }
}

void CheckTargetValid::execute(PathPlanningData &data)
{
    MRA::Geometry::Position target = data.getSubTarget();

    // check option: what to do if target is inside a forbidden area
    auto mode = data.configPP.boundaries.targetInsideForbiddenArea;
    if (mode != BoundaryOptionEnum::ALLOW)
    {
        for (auto it = data.forbiddenAreas.begin(); it != data.forbiddenAreas.end(); ++it)
        {
            if (it->isPointInside(vec2d(target.x, target.y)))
            {
                // JPGL - 2022-06-27
                //      This trace was converted to an event log warning in order to
                //      monitor how often we need to draw a forbidden area on the ball
                //      We might want to consider putting this back to TRACE if it spams
                //      th event logs too much
                MRA_LOG_WARNING("target is in forbidden area id=%d", it->id);
                if (mode == BoundaryOptionEnum::CLIP)
                {
                    // TODO: implement clipping option (geometry)
                    MRA_LOG_WARNING("target clipping to forbidden area is not yet implemented");
                    mode = BoundaryOptionEnum::STOP_AND_FAIL;
                }
                // handle the violation
                handleViolation(data, mode);
            }
        }
    }

    // allow target position in TTA for park/substitute
    bool targetInsideTTA = false;
    if (cEnvironmentField::getInstance().isPositionInArea(target.x, target.y, A_TTA))
    {
        targetInsideTTA = true;
    }

    // check option: what to do if target is outside of field
    mode = data.configPP.boundaries.targetOutsideField;
    if (mode != BoundaryOptionEnum::ALLOW)
    {
        double fieldMarginX = data.configPP.boundaries.fieldMarginX;
        double fieldMarginY = data.configPP.boundaries.fieldMarginY;
        double fieldLength = cEnvironmentField::getInstance().getLength();
        double fieldWidth = cEnvironmentField::getInstance().getWidth();
        double limit = fieldWidth * 0.5 + fieldMarginX;
        if (fabs(target.x) > limit && !targetInsideTTA)
        {
            MRA_LOG_DEBUG("target field dimension X violation (limit=%6.2f)", limit);
            handleViolation(data, mode);
            if (mode == BoundaryOptionEnum::CLIP)
            {
                double tmp = std::clamp(target.x, 0.0, limit);
                data.path[0].pos.x = tmp;
            }
        }
        limit = fieldLength * 0.5 + fieldMarginY;
        if (fabs(target.y) > limit && !targetInsideTTA)
        {
            MRA_LOG_DEBUG("target field dimension Y violation (limit=%6.2f)", limit);
            handleViolation(data, mode);
            if (mode == BoundaryOptionEnum::CLIP)
            {
                double tmp = std::clamp(target.y, 0.0, limit);
                data.path[0].pos.y = tmp;
            }
        }
    }

    // check options: own and opponent halves
    // (especially the CLIP options are useful for playing on half a demo field)
    mode = data.configPP.boundaries.targetOnOwnHalf;
    if (mode != BoundaryOptionEnum::ALLOW)
    {
        if (target.y < 0.0)
        {
            MRA_LOG_DEBUG("target on own half not allowed");
            handleViolation(data, mode);
            if (mode == BoundaryOptionEnum::CLIP)
            {
                data.path[0].pos.y = 0.0;
            }
        }
    }
    mode = data.configPP.boundaries.targetOnOpponentHalf;
    if (mode != BoundaryOptionEnum::ALLOW)
    {
        if (target.y > 0.0)
        {
            MRA_LOG_DEBUG("target on opponent half not allowed");
            handleViolation(data, mode);
            if (mode == BoundaryOptionEnum::CLIP)
            {
                data.path[0].pos.y = 0.0;
            }
        }
    }
}

