#include "../include/teaming_planner/teaming_planner.h"

void TeamingPlanner::moduleLoopCallback(const ros::TimerEvent& event)
{
    teamingPlannerMain();
}