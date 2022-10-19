#include "../include/teaming_planner/teaming_planner.h"

bool TeamingPlanner::getGoTherePath_cp(std::vector<DistributedGlobalPathPlanner::Common::Pose>& goTherePath)
{
    bool status = true;

    std::cout << "IIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIIII" << std::endl;
    std::cout << "get go there path phase sync call back" << std::endl;

    for (auto pose : mGoTherePath_cp)
    {
        std::cout << "pose x: " << pose.position(0) << " pose y: " << pose.position(1) << " pose z: " << pose.position(2) << std::endl;
    }

    if (!mGoTherePath_cp.empty())
    {
        std::vector<DistributedGlobalPathPlanner::Common::Pose> tmp_to_send;
        tmp_to_send.push_back(mGoTherePath_cp.front());
        goTherePath = tmp_to_send;
        ROS_INFO("goTherePath of size %i is sent, goTherePath x: %f", goTherePath.size(), goTherePath.front().position(0));
    }
    else
    {
        goTherePath = mGoTherePath_cp;
        status = false;
        ROS_WARN("Agent%d CPH: get mGoTherePath_cp empty!", mSourceSegmentId);
    }


    return status;
}

bool TeamingPlanner::getPhasesAndTimeRecordOfAgents_cp(std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PhaseAndTime>& phasesAndTimeRecordOfAgents)
{
    bool status = true;
    ROS_WARN("Get PhasesAndTimeRecord of Agents CP is called!");

    if (!mAgentsPhasesAndTimeMap_cp.empty())
    {
        std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PhaseAndTime> tmp;
        ROS_INFO("Agent%d CP: mAgentsInTeamVector is of size %d", mSourceSegmentId, mAgentsInTeamVector.size());
        for (auto agentnumber : mAgentsInTeamVector)
        {
            if (mAgentsPhasesAndTimeMap_cp.find(agentnumber) != mAgentsPhasesAndTimeMap_cp.end())
            {   
                tmp[agentnumber] = mAgentsPhasesAndTimeMap_cp[agentnumber];
            }
        }
        phasesAndTimeRecordOfAgents = tmp;
        ROS_INFO("Agent%d CP: Passing PhasesAndTime record of size %d", mSourceSegmentId, phasesAndTimeRecordOfAgents.size());
    }
    else
    {
        ROS_WARN("Agent%d CP: get mAgentsPhasesAndTimeMap_cp empty!", mSourceSegmentId);
        phasesAndTimeRecordOfAgents = mAgentsPhasesAndTimeMap_cp;
    }
    return status;
}

bool TeamingPlanner::getOwnAgentPose_cp(DistributedGlobalPathPlanner::Common::Pose& ownAgentPose)
{
    bool status = true;

    ownAgentPose = mOwnAgentPose_cp;

    return status;
}

bool TeamingPlanner::pubOwnPhaseAndTime_cp(const int32_t aAgentId, const DistributedGlobalPathPlanner::Common::PhaseAndTime ownAgentPhaseAndTime)
{
    bool status = true;
    status = status && aAgentId == mSourceSegmentId;
    if (status)
    {
        mt_msgs::phaseAndTime tmp;
        tmp.header.stamp = ros::Time::now();
        tmp.sourceSegmentId = mSourceSegmentId;
        tmp.phase = static_cast<uint8_t>(ownAgentPhaseAndTime.phase);
        tmp.time = ownAgentPhaseAndTime.timeMicroSecs;

        mPhaseAndTimePublisher_cp.publish(tmp);
    }
    else 
    {
        ROS_ERROR("[TeamingPlanner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }

    return status;
}

bool TeamingPlanner::pubOwnPoseFunc_cp(const int32_t aAgentId, const DistributedGlobalPathPlanner::Common::Pose ownAgentPose)
{
    bool status = true;
    status = status && aAgentId == mSourceSegmentId;
    if (status)
    {
        mt_msgs::pose tmp;
        tmp.header.stamp = ros::Time::now();
        tmp.sourceSegmentId = mSourceSegmentId;
        tmp.position.x = ownAgentPose.position(0);
        tmp.position.y = ownAgentPose.position(1);
        tmp.position.z = ownAgentPose.position(2);
        tmp.headingRad = ownAgentPose.headingRad;

        mPosePublisher_cp.publish(tmp);
    }
    else
    {
        ROS_ERROR("[Teaming Planner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;
}

bool TeamingPlanner::getAgentsPose_cp(std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::Pose>& agentsPose)
{
    bool status = true;
    
    if (!mAgentsPoseMap_cp.empty())
    {
        std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::Pose> tmp;
        for (auto agentnumber : mAgentsInTeamVector)
        {
            if (mAgentsPoseMap_cp.find(agentnumber) != mAgentsPoseMap_cp.end())
            {   
                tmp[agentnumber] = mAgentsPoseMap_cp[agentnumber];
            }
            ROS_INFO("[Teaming Planner %d]: tmpmmAgentsPoseMap_cp contains %d", mSourceSegmentId, agentnumber);
        }

        // agentsPose = mAgentsPoseMap_cp;
        agentsPose = tmp;
    }
    else
    {
        status = false;
        agentsPose = mAgentsPoseMap_cp;
        ROS_WARN("Agent%d CPH: get mAgentsPoseMap_cp empty!", mSourceSegmentId);
    }

    return status;
}


bool TeamingPlanner::getAgentsPathAndWaypointProgress_cp(std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PathAndWaypointProgress>& agentsGoTherePathAndWaypointProgress)
{
    bool status = true;

    if (!mAgentsPathAndWaypointProgressMap_cp.empty())
    {
        std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PathAndWaypointProgress>  tmp;
        for (auto agentnumber : mAgentsInTeamVector)
        {
            if (mAgentsPathAndWaypointProgressMap_cp.find(agentnumber) != mAgentsPathAndWaypointProgressMap_cp.end())
            {
                tmp[agentnumber] = mAgentsPathAndWaypointProgressMap_cp[agentnumber];
            }
        }
        // agentsGoTherePathAndWaypointProgress = mAgentsPathAndWaypointProgressMap_cp;
        agentsGoTherePathAndWaypointProgress = tmp;

        for ( auto pathWayPoint : agentsGoTherePathAndWaypointProgress)
        {
            std::cout << "[Teaming Planner " << mSourceSegmentId << "] Agent ID path: "  << pathWayPoint.first << std::endl;
            for ( auto point : pathWayPoint.second.poses)
            {
                std::cout << "Path: " << " Poses are  x: " << point.position.x() << " y: " << point.position.y() << " z: " << point.position.z() << std::endl;
            }
        }

        
    }
    else
    {
        status = false;
        agentsGoTherePathAndWaypointProgress = mAgentsPathAndWaypointProgressMap_cp;
        ROS_WARN("Agent%d CPH: get mAgentsPathAndWaypointProgressMap_cp empty!", mSourceSegmentId);
    }

    return status;
}

bool TeamingPlanner::pubOwnPathAndWaypointProgress_cp(const int32_t aAgentId, const DistributedGlobalPathPlanner::Common::PathAndWaypointProgress goTherePathAndWaypointProgress)
{
    bool status = true;

    if (status)
    {
        mt_msgs::pathAndProgress tmp_publish;
        tmp_publish.header.stamp = ros::Time::now();
        tmp_publish.waypointProgress = goTherePathAndWaypointProgress.waypointProgress;
        tmp_publish.sourceSegmentId = aAgentId;

        // Copy over the poses
        std::vector<mt_msgs::pose> pose_tmp_vector;
        for (DistributedGlobalPathPlanner::Common::Pose pose : goTherePathAndWaypointProgress.poses)
        {
            mt_msgs::pose tmp;
            tmp.position.x = pose.position(0);
            tmp.position.y = pose.position(1);
            tmp.position.z = pose.position(2);
            pose_tmp_vector.push_back(tmp);
        }
        tmp_publish.poseVector = pose_tmp_vector;
        mPathAndOwnWayPointProgressPublisher_cp.publish(tmp_publish);
    }

    return status;
}


bool TeamingPlanner::getAgentsPlannedPath_cp(std::unordered_map<int32_t, std::vector<Eigen::Vector3d>>& agentsPlannedPath)
{
    bool status = true;

    if (!mAgentsPlannedPathMap_cp.empty())
    {
        std::unordered_map<int32_t, std::vector<Eigen::Vector3d>>  tmp;
        for (auto agentnumber : mAgentsInTeamVector)
        {
            if (mAgentsPlannedPathMap_cp.find(agentnumber) != mAgentsPlannedPathMap_cp.end())
            {
                tmp[agentnumber] = mAgentsPlannedPathMap_cp[agentnumber];
            }
        }

        // agentsPlannedPath = mAgentsPlannedPathMap_cp;
        agentsPlannedPath = tmp;
    }
    else
    {
        status = false;
        ROS_WARN("Agent%d CPH: get mAgentsPlannedPathMap_cp empty!", mSourceSegmentId);
    }
    return status;
}

bool TeamingPlanner::pubOwnPlannedPath_cp(const int32_t aAgentId, const std::vector<Eigen::Vector3d> ownPlannedPath)
{
    bool status = true;

    if (status)
    {
        mt_msgs::posevector tmp_vec_msg;
        std::vector<mt_msgs::pose> tmp_vec_to_be_assigned;
        ROS_INFO("Agent %i: Planned path size of %i", mSourceSegmentId, ownPlannedPath.size());
        for (Eigen::Vector3d path: ownPlannedPath)
        {
            mt_msgs::pose tmp;
            tmp.position.x = path(0);
            tmp.position.y = path(1);
            tmp.position.z = path(2);
            tmp_vec_to_be_assigned.push_back(tmp);
        }
        tmp_vec_msg.poseVector = tmp_vec_to_be_assigned;
        tmp_vec_msg.header.stamp = ros::Time::now();
        tmp_vec_msg.sourceSegmentId = aAgentId;
        mOwnPlannedPathPublisher_cp.publish(tmp_vec_msg);
    }
    return status;
}


bool TeamingPlanner::getAgentsProcessedPathOfAgents_cp(std::unordered_map<int32_t, std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PathAndCost>>& agentsProcessedPathOfAgents)
{
    bool status = true;

    if (!mAgentsProcessedPathOfAgentsMap_cp.empty())
    {
        std::unordered_map<int32_t, std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PathAndCost>>   tmp;
        for (auto agentnumber : mAgentsInTeamVector)
        {
            if (mAgentsProcessedPathOfAgentsMap_cp.find(agentnumber) != mAgentsProcessedPathOfAgentsMap_cp.end())
            {
                tmp[agentnumber] = mAgentsProcessedPathOfAgentsMap_cp[agentnumber];
            }
        }

        // agentsProcessedPathOfAgents = mAgentsProcessedPathOfAgentsMap_cp;
        agentsProcessedPathOfAgents = tmp;

    }
    else
    {
        status = false;
        ROS_WARN("Agent%d CPH: get mAgentsProcessedPathOfAgentsMap_cp empty!", mSourceSegmentId);
    }

    return status;
}

bool TeamingPlanner::pubOwnProcessedPathOfAgents_cp(const int32_t aAgentId, const std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PathAndCost> ownProcessedPathOfAgents)
{
    bool status = true;

    mt_msgs::pathAndCostVector tmp_publish;
    std::vector<mt_msgs::pathAndCost> path_and_cost_vec_tmp;
    for (auto const processedPath : ownProcessedPathOfAgents)
    {
        // for each agent, get all the paths and cost
        mt_msgs::pathAndCost path_and_cost_tmp;
        std::vector<mt_msgs::pose> pose_vector_tmp;

        // Copy out the poses
        for (auto position : processedPath.second.positions)
        {
            mt_msgs::pose pose_tmp;
            pose_tmp.position.x = position(0);
            pose_tmp.position.y = position(1);
            pose_tmp.position.z = position(2);
            pose_vector_tmp.push_back(pose_tmp);
        }
        path_and_cost_tmp.poseVector = pose_vector_tmp;
        path_and_cost_tmp.cost = processedPath.second.cost;
        path_and_cost_tmp.sourceSegmentId = processedPath.first;
        path_and_cost_vec_tmp.push_back(path_and_cost_tmp);
    }
    tmp_publish.pathAndCostVector = path_and_cost_vec_tmp;
    tmp_publish.sourceSegmentId = aAgentId;
    tmp_publish.header.stamp = ros::Time::now();

    if (!tmp_publish.pathAndCostVector.empty())
    {
        mOwnProcessedPathOfAgentsPublisher_cp.publish(tmp_publish);
    }
    else
    {
        status = false;
        ROS_INFO("Teaming Plannner %i, unable to publish processed path of all Agents as they are empty!", mSourceSegmentId);
    }

    return status;
}


bool TeamingPlanner::getAgentsBestProcessedPath_cp(std::unordered_map<int32_t, std::vector<Eigen::Vector3d>>& agentsBestProcessedPath)
{
    bool status = true;

    if (!mAgentsBestProcessedPath_cp.empty())
    {
        std::unordered_map<int32_t, std::vector<Eigen::Vector3d>>   tmp;

        for (auto agentnumber : mAgentsInTeamVector)
        {
            if (mAgentsBestProcessedPath_cp.find(agentnumber) != mAgentsBestProcessedPath_cp.end())
            {
                tmp[agentnumber] = mAgentsBestProcessedPath_cp[agentnumber];
            }
        }
        // agentsBestProcessedPath = mAgentsBestProcessedPath_cp;
        agentsBestProcessedPath = tmp;
    }
    else
    {
        status = false;
        ROS_WARN("Agent%d CPH: get mAgentsBestProcessedPath_cp empty!", mSourceSegmentId);
    }

    return status;
}


bool TeamingPlanner::pubOwnBestProcessedPath_cp(const int32_t aAgentId, const std::vector<Eigen::Vector3d> ownBestProcessedPath)
{
    bool status = true;
    mt_msgs::posevector tmp_vector;
    for (auto path : ownBestProcessedPath)
    {
        mt_msgs::pose tmp;
        tmp.position.x = path(0);
        tmp.position.y = path(1);
        tmp.position.z = path(2);
        tmp_vector.poseVector.push_back(tmp);
    }

    tmp_vector.header.stamp = ros::Time::now();
    tmp_vector.sourceSegmentId = aAgentId;

    if (!tmp_vector.poseVector.empty())
    {
        mOwnBestProcessedPath_cpPublisher_cp.publish(tmp_vector);
        status = status && true;
    }
    else
    {
        ROS_WARN("[Teaming Planner %d]: OwnBestProcessedPath is empty\n", mSourceSegmentId);
    }

    return status;
}

bool TeamingPlanner::pubProcessedGoTherePath_cp(const int32_t aAgentId, const std::vector<DistributedGlobalPathPlanner::Common::Pose> processedGoTherePath)
{
    bool status = true;
    
    geometry_msgs::PoseArray tmp_pose_array;

    ROS_INFO("Teaming Planner %d]: Publishing processedGoTherePath size is %i", mSourceSegmentId, processedGoTherePath.size());

    for (auto path : processedGoTherePath)
    {
        geometry_msgs::Pose tmp;
        tmp.position.x = path.position(0);
        tmp.position.y = path.position(1);
        tmp.position.z = path.position(2);
        tmp.orientation.x = 0;
        tmp.orientation.y = 0;
        tmp.orientation.z = 0;
        tmp.orientation.w = 1;

        tmp_pose_array.poses.push_back(tmp);
    }

    tmp_pose_array.header.stamp = ros::Time::now();
    tmp_pose_array.header.frame_id = "/odom";

    if (!tmp_pose_array.poses.empty())
    {
        mProcessedGoTherePathPublisher_cp.publish(tmp_pose_array);
        status = status && true;
    }
    else
    {
        status = false;
        ROS_WARN("[Teaming Planner %d]: processedGoTherePath is empty\n", mSourceSegmentId);
    }

    return status;
}


void TeamingPlanner::clearAgentsPoseBuffer_cp()
{
    mAgentsPoseMap_cp.clear();
}

void TeamingPlanner::clearAgentsPathAndWaypointProgressBuffer_cp()
{
    mAgentsPathAndWaypointProgressMap_cp.clear();
}

void TeamingPlanner::clearAgentsPlannedPathBuffer_cp()
{
    mAgentsPlannedPathMap_cp.clear();
}

void TeamingPlanner::clearAgentsProcessedPathOfAgentsBuffer_cp()
{
    mAgentsProcessedPathOfAgentsMap_cp.clear();
}

void TeamingPlanner::clearAgentsBestProcessedPathBuffer_cp()
{
    mAgentsBestProcessedPath_cp.clear();
}

void TeamingPlanner::clearPhasesAndTime_cp()
{
    mAgentsPhasesAndTimeMap_cp.clear();
}