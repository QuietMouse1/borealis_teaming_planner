#include "../include/teaming_planner/teaming_planner.h"

bool TeamingPlanner::getOwnAgentId(int32_t& ownAgentID)
{
    bool status = true;

    ownAgentID = mSourceSegmentId;

    return status;
}

bool TeamingPlanner::getNumberOfAgentsInTeam(int32_t& numberOfAgentsInTeam)
{
    bool status = true;

    numberOfAgentsInTeam = mTeamSize;

    return status;
}

bool TeamingPlanner::getOwnAgentLidarPointCloud(sensor_msgs::PointCloud& cloud)
{

    bool status = false;
    if (!mSystemPointCloud.points.empty())
    {
        cloud = mSystemPointCloud;
        status = true;
    }
    else
    {
        ROS_WARN("[Teaming Planner %d]: Agents Point Cloud empty", mSourceSegmentId);
        status = false;
    }
    
    return status;
}