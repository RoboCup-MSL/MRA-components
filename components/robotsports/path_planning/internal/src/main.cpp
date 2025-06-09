/*
 * main.cpp
 *
 *  Created on: July, 2019
 *      Author: Jan Feitsma
 */


#include "PathPlanningClient.hpp"

int main(int argc, char **argv)
{
    // INIT_TRACE("pathPlanning");

    PathPlanningClient client;
    client.spin();

    return 0;
}
