/*
 * PathPlanningClient.hpp
 *
 *  Created on: December, 2019
 *      Author: Jan Feitsma
 */

#ifndef PATHPLANNINGCLIENT_HPP_
#define PATHPLANNINGCLIENT_HPP_

#include "PathPlanning.hpp" // for enum actionResultTypeEnum


class PathPlanningClient
{
public:
    PathPlanningClient();
    virtual ~PathPlanningClient();

    // full iteration:
    // * get RTDB inputs
    // * calculate
    // * set RTDB outputs
    // * return status as actionResultTypeEnum
    virtual actionResultTypeEnum iterate();

    // legacy spinner, which was used when PathPlanning had its own process,
    // before being integrated into motionPlanning as library
    void spin();

private:
};

#endif

