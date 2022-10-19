#include "../include/teaming_planner/teaming_planner.h"

// Functions decleration for the formation handler pointers

bool TeamingPlanner::pubPhaseAndTime_rf(const int32_t aAgentId, const DistributedFormation::Common::PhaseAndTime aPhaseAndTime)
{
    bool status = true;

    status = status && aAgentId == mSourceSegmentId;
    if (status)
    {
        mt_msgs::phaseAndTime tmp;
        tmp.header.stamp = ros::Time::now();
        tmp.sourceSegmentId = mSourceSegmentId;
        tmp.phase = static_cast<uint8_t>(aPhaseAndTime.phase);
        tmp.time = aPhaseAndTime.timeMicroSecs;

        mPhaseAndTimePublisher_rf.publish(tmp);
    }
    else 
    {
        ROS_ERROR("[TeamingPlanner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;
}

bool TeamingPlanner::pubPose_rf(const int32_t aAgentId, const DistributedFormation::Common::Pose aPose)
{
    bool status = true;

    status = status && aAgentId == mSourceSegmentId;
    if (status)
    {
        mt_msgs::pose tmp;
        tmp.header.stamp = ros::Time::now();
        tmp.sourceSegmentId = mSourceSegmentId;
        tmp.position.x = aPose.position.x;
        tmp.position.y = aPose.position.y;
        tmp.position.z = aPose.position.z;
        tmp.headingRad = aPose.headingRad;

        mPosePublisher_rf.publish(tmp);
    }
    else
    {
        ROS_ERROR("[Teaming Planner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;
}

bool TeamingPlanner::pubDirectionUtility_rf(const int32_t aAgentId, const DistributedFormation::Common::DirectionUtility aDirectionUtility)
{
    bool status = true;
    
    status = status && aAgentId == mSourceSegmentId;
    if (status)
    {
        mt_msgs::angleIndexAndUtility tmp;
        tmp.header.stamp = ros::Time::now();
        tmp.sourceSegmentId = mSourceSegmentId;
        tmp.angleIndexAndUtility = aDirectionUtility.angleIndexAndUtility;

        mDirectionUtilityPublisher_rf.publish(tmp);
    }
    else
    {
        ROS_ERROR("[Teaming Planner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;
}

bool TeamingPlanner::pubConvexRegion2D_rf(const int32_t aAgentId, const DistributedFormation::Common::ConvexRegion2D aConvexRegion2D)
{
    bool status = true;
    
    status = status && aAgentId == mSourceSegmentId;
    if (status)
    {
        mt_msgs::convexRegion2D tmp;
        tmp.header.stamp = ros::Time::now();
        tmp.sourceSegmentId = mSourceSegmentId;

        Eigen::VectorXd matrixACol1Eigen = aConvexRegion2D.A.col(0);
        Eigen::VectorXd matrixACol2Eigen = aConvexRegion2D.A.col(1);
        
        if (matrixACol1Eigen.size() != 0 && matrixACol2Eigen.size() != 0 && aConvexRegion2D.b.size() != 0)
        {
            // only publish if they are not empty
            std::vector<double> matrixACol1(matrixACol1Eigen.data(), matrixACol1Eigen.data() + matrixACol1Eigen.size());
            tmp.matrixACol1 = matrixACol1;
            std::vector<double> matrixACol2(matrixACol2Eigen.data(), matrixACol2Eigen.data() + matrixACol2Eigen.size());
            tmp.matrixACol2 = matrixACol2;
            std::vector<double> matrixB(aConvexRegion2D.b.data(), aConvexRegion2D.b.data() + aConvexRegion2D.b.size());
            tmp.matrixB = matrixB;
            mConvexRegion2DPublisher_rf.publish(tmp);
        }

        // if (mDebugVerbose)
        // {
        //     ROS_INFO("pubConvexRegion2D_rf Vector A column 1 Values: ");
        //     for (auto value : matrixACol1)
        //     {
        //         ROS_INFO("%d ", value);
        //     }
        //     ROS_INFO("\n");

        //     ROS_INFO("pubConvexRegion2D_rf Vector A column 2 Values: ");
        //     for (auto value : matrixACol2)
        //     {
        //         ROS_INFO("%d ", value);
        //     }
        //     ROS_INFO("\n");

        //     ROS_INFO("pubConvexRegion2D_rf Matrix A Values: ");

        //     ROS_INFO("pubConvexRegion2D_rf Vector B Values: ");
        //     for (auto value : matrixB)
        //     {
        //         ROS_INFO("%d ", value);
        //     }
        //     ROS_INFO("\n");

        //     ROS_INFO("pubConvexRegion2D_rf Matrix B Values: ");
        // }
    }
    else
    {
        ROS_ERROR("[Teaming Planner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;
}

bool TeamingPlanner::pubConvexRegion3D_rf(const int32_t aAgentId, const DistributedFormation::Common::ConvexRegion3D aConvexRegion3D)
{
    bool status = true;
    
    status = status && aAgentId == mSourceSegmentId;
    if (status)
    {
        mt_msgs::convexRegion3D tmp;
        tmp.header.stamp = ros::Time::now();
        tmp.sourceSegmentId = mSourceSegmentId;
        Eigen::VectorXd matrixACol1Eigen = aConvexRegion3D.A.col(0);
        std::vector<double> matrixACol1(matrixACol1Eigen.data(), matrixACol1Eigen.data() + matrixACol1Eigen.size());
        tmp.matrixACol1 = matrixACol1;
        Eigen::VectorXd matrixACol2Eigen = aConvexRegion3D.A.col(1);
        std::vector<double> matrixACol2(matrixACol2Eigen.data(), matrixACol2Eigen.data() + matrixACol2Eigen.size());
        tmp.matrixACol2 = matrixACol2;
        Eigen::VectorXd matrixACol3Eigen = aConvexRegion3D.A.col(2);
        std::vector<double> matrixACol3(matrixACol3Eigen.data(), matrixACol3Eigen.data() + matrixACol3Eigen.size());
        tmp.matrixACol3 = matrixACol3;
        std::vector<double> matrixB(aConvexRegion3D.b.data(), aConvexRegion3D.b.data() + aConvexRegion3D.b.size());
        tmp.matrixB = matrixB;
        
        mConvexRegion3DPublisher_rf.publish(tmp);

        // if (mDebugVerbose)
        // {
        //     ROS_INFO("pubConvexRegion3D_rf Vector A column 1 Values: ");
        //     for (auto value : matrixACol1)
        //     {
        //         ROS_INFO("%d ", value);
        //     }
        //     ROS_INFO("\n");

        //     ROS_INFO("pubConvexRegion3D_rf Vector A column 2 Values: ");
        //     for (auto value : matrixACol2)
        //     {
        //         ROS_INFO("%d ", value);
        //     }
        //     ROS_INFO("\n");

        //     ROS_INFO("pubConvexRegion3D_rf Vector A column 3 Values: ");
        //     for (auto value : matrixACol3)
        //     {
        //         ROS_INFO("%d ", value);
        //     }
        //     ROS_INFO("\n");

        //     ROS_INFO("pubConvexRegion3D_rf Vector B Values: ");
        //     for (auto value : matrixB)
        //     {
        //         ROS_INFO("%d ", value);
        //     }
        //     ROS_INFO("\n");
        // }
    }
    else
    {
        ROS_ERROR("[Teaming Planner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;
}

// Published Assigned Pose of the Agent into the /t265_pose_frame
bool TeamingPlanner::pubAssignedPose_rf(const int32_t aAgentId, const DistributedFormation::Common::Pose aAssignedVirtualPose)
{
    bool status = true;

    status = (status && aAgentId == mSourceSegmentId);
    if (status)
    {
        geometry_msgs::PoseStamped tmp;
        tmp.header.frame_id = "/odom";
        
        tmp.pose.position.x = aAssignedVirtualPose.position.x;
        tmp.pose.position.y = aAssignedVirtualPose.position.y;
        tmp.pose.position.z = aAssignedVirtualPose.position.z;

        tf::Quaternion tQuat = tf::createQuaternionFromYaw(aAssignedVirtualPose.headingRad);

        tmp.pose.orientation.w = tQuat.getW();
        tmp.pose.orientation.x = tQuat.getX();
        tmp.pose.orientation.y = tQuat.getY();
        tmp.pose.orientation.z = tQuat.getZ();

        std::string systemFrame = "/odom";
        // std::string targetFrame = "uav" + std::to_string(mSourceSegmentId) + "/t265_pose_frame";

        ros::Time tm;
        std::string err_string;

        mAssignedVirtualPosePublisher_rf.publish(tmp);
    }

    else
    {
        ROS_ERROR("[Teaming Planner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;
}

bool TeamingPlanner::pubAssignedPoseMap_rf(const int32_t aAgentId, const std::unordered_map<int32_t, DistributedFormation::Common::Pose> aAssignedPoseMap)
{
    bool status = true;

    status = status && aAgentId == mSourceSegmentId;

    if (status)
    {
        mt_msgs::posevector tmpPoseVector;
        tmpPoseVector.header.stamp = ros::Time::now();
        tmpPoseVector.sourceSegmentId = mSourceSegmentId;
        
        for (auto assignedPose : aAssignedPoseMap)
        {
            mt_msgs::pose tmpPose;
            tmpPose.position.x = assignedPose.second.position.x;
            tmpPose.position.y = assignedPose.second.position.y;
            tmpPose.position.z = assignedPose.second.position.z;

            tmpPose.headingRad = assignedPose.second.headingRad;
            
            tmpPose.sourceSegmentId = assignedPose.first;

            tmpPose.header.stamp = ros::Time::now();

            tmpPoseVector.poseVector.emplace_back(tmpPose);
        }

        if (!tmpPoseVector.poseVector.empty())
        {
            mAssignedVirtualPoseMapPublisher_rf.publish(tmpPoseVector);
            status = status && true;
        }
        else
        {
            status = false;
            ROS_WARN("[Teaming Planner %d]: Assigned Virtual Pose Map empty\n", mSourceSegmentId);

        }
    }
    else
    {
        ROS_ERROR("[Teaming Planner %d]: Agent ID %d and Source Segment ID %d mismatch\n", mSourceSegmentId, aAgentId, mSourceSegmentId);
    }
    return status;
}

bool TeamingPlanner::getPosesForFormationToTrack_rf(std::vector<DistributedFormation::Common::Pose>& posesForFormationToTrack)
{
    bool status(false);

    if (mTask.type == Common::Entity::MTTaskEnum::FOLLOW_ME )
    {
        if (!mHistoryOfHumanPoses_rf.empty())
        {
            posesForFormationToTrack = mHistoryOfHumanPoses_rf;
            status = true;
            mHistoryOfHumanPosesReceived = false;
            ROS_INFO("///////////////////////////////////////////////////////////////////////////////////////////////////");
            for (auto pose : mHistoryOfHumanPoses_rf)
            {
                ROS_INFO("[Teaming Planner %d]", mSourceSegmentId);
                std::cout << "pose x: " << pose.position.x << " pose y: " << pose.position.y << " pose z: " << pose.position.z << std::endl;
            }
            ROS_INFO("///////////////////////////////////////////////////////////////////////////////////////////////////");
        }
        else
        {
            status = false;
            ROS_INFO("History of human poses is empty!");
        }
    }
    
    if (mTask.type == Common::Entity::MTTaskEnum::GO_THERE)
    {
        if (!mProcessedGoTherePath.empty())
        {
            // mGoTherePath_cp
            // posesForFormationToTrack = ;
            posesForFormationToTrack = mProcessedGoTherePath;
            status = true;
        }
        else
        {
            posesForFormationToTrack = mProcessedGoTherePath;
            status = false;
            ROS_ERROR("[Teaming Planner %d]: No go there poses for formation to track! \n", mSourceSegmentId);
        }
    }

    return status;
}

bool TeamingPlanner::getPhaseAndTimeMap_rf(std::unordered_map<int32_t, DistributedFormation::Common::PhaseAndTime>& phaseAndTimeMap)
{
    bool status = true;
    ROS_WARN("Get getPhaseAndTimeMap_rf of Agents CP is called!");

    if (!mAgentsPhaseAndTimeMap_rf.empty())
    {

        std::unordered_map<int32_t, DistributedFormation::Common::PhaseAndTime> tmp;
        for (auto agentnumber : mAgentsInTeamVector)
        {
            if (mAgentsPhaseAndTimeMap_rf.find(agentnumber) != mAgentsPhaseAndTimeMap_rf.end())
            {
                tmp[agentnumber] = mAgentsPhaseAndTimeMap_rf[agentnumber];
            }
        }
        phaseAndTimeMap = tmp;
    }
    else
    {
        // phaseAndTimeMap.clear();
        phaseAndTimeMap = mAgentsPhaseAndTimeMap_rf;
        ROS_WARN("[Teaming Planner %d]: Agents Phase and Time map empty", mSourceSegmentId);
        status = false;
    }
    return status;
}

bool TeamingPlanner::getPoseMap_rf(std::unordered_map<int32_t, DistributedFormation::Common::Pose>& poseMap)
{
    bool status = true;

    if (!mAgentsPoseMap_rf.empty())
    {
        std::unordered_map<int32_t, DistributedFormation::Common::Pose> tmp;

        ROS_INFO("[Teaming Planner %d]: passing Pose map into phase sync", mSourceSegmentId);
        for (auto agentnumber : mAgentsInTeamVector)
        {
            if (mAgentsPoseMap_rf.find(agentnumber) != mAgentsPoseMap_rf.end())
            {
                tmp[agentnumber] = mAgentsPoseMap_rf[agentnumber];
            }
            std::cout << "Pose map contains agent " << agentnumber << std::endl;
        }

        // poseMap = mAgentsPoseMap_rf;
        poseMap = tmp;
    }
    else
    {
        // poseMap.clear();
        poseMap = mAgentsPoseMap_rf;
        ROS_WARN("[Teaming Planner %d]: Agents Position map empty", mSourceSegmentId);
        status = false;
    }
    return status;
}

bool TeamingPlanner::getDirectionUtilityMap_rf(std::unordered_map<int32_t, DistributedFormation::Common::DirectionUtility>& directionUtilityMap)
{
    bool status = true;

    if (!mAgentsDirectionUtilityMap_rf.empty())
    {
        std::unordered_map<int32_t, DistributedFormation::Common::DirectionUtility> tmp;

        // mAgentsDirectionUtilityMap_rf this guy could only have 2 and 3
        // if mAgentsInTeamVector contains 1 2 3
        // tmp[1] = mAgentsDirectionUtilityMap_rf[1]; <-- causes garbage value in tmp[1]

        for (auto agentnumber : mAgentsInTeamVector)
        {
            if (mAgentsDirectionUtilityMap_rf.find(agentnumber) != mAgentsDirectionUtilityMap_rf.end())
            {
                tmp[agentnumber] = mAgentsDirectionUtilityMap_rf[agentnumber];
            }
        }
        directionUtilityMap = tmp;
        // directionUtilityMap = mAgentsDirectionUtilityMap_rf;
    }
    else
    {
        ROS_WARN("[Teaming Planner %d]: Agents Direction Utility map empty", mSourceSegmentId);
        status = false;
    }
    return status;
}

bool TeamingPlanner::getConvexRegion2DMap_rf(std::unordered_map<int32_t, DistributedFormation::Common::ConvexRegion2D>& convexRegion2DMap)
{
    bool status = true;

    if (!mAgentsConvexRegion2DMap_rf.empty())
    {
        std::unordered_map<int32_t, DistributedFormation::Common::ConvexRegion2D>  tmp;

        for (auto agentnumber : mAgentsInTeamVector)
        {
            if (mAgentsConvexRegion2DMap_rf.find(agentnumber) != mAgentsConvexRegion2DMap_rf.end())
            {
                tmp[agentnumber] = mAgentsConvexRegion2DMap_rf[agentnumber];

            }
        }
        // convexRegion2DMap = mAgentsConvexRegion2DMap_rf;
        convexRegion2DMap = tmp;

    }
    else
    {
        // convexRegion2DMap.clear();
        ROS_WARN("[Teaming Planner %d]: Agents Convex Region 2D map empty", mSourceSegmentId);
        status = false;
    }
    return status;
}

bool TeamingPlanner::getConvexRegion3DMap_rf(std::unordered_map<int32_t, DistributedFormation::Common::ConvexRegion3D>& convexRegion3DMap)
{
    bool status = true;

    if (!mAgentsConvexRegion3DMap_rf.empty())
    {
        std::unordered_map<int32_t, DistributedFormation::Common::ConvexRegion3D>  tmp;

        for (auto agentnumber : mAgentsInTeamVector)
        {
            if (mAgentsConvexRegion3DMap_rf.find(agentnumber) != mAgentsConvexRegion3DMap_rf.end())
            {
                tmp[agentnumber] = mAgentsConvexRegion3DMap_rf[agentnumber];
            }
        }
        convexRegion3DMap = tmp;
        // convexRegion3DMap = mAgentsConvexRegion3DMap_rf;
    }
    else
    {
        ROS_WARN("[Teaming Planner %d]: Agents Convex Region 3D map empty", mSourceSegmentId);
        status = false;
    }
    return status;
}

bool TeamingPlanner::getAssignedVirtualPoseMap_rf(std::unordered_map<int32_t, std::unordered_map<int32_t, DistributedFormation::Common::Pose>>& assignedVirtualPoseMap)
{
    bool status = true;

    if (!mAgentsAssignedVirtualPoseMap_rf.empty())
    {
        std::unordered_map<int32_t, std::unordered_map<int32_t, DistributedFormation::Common::Pose>> tmp;

        for (auto agentnumber : mAgentsInTeamVector)
        {
            if (mAgentsAssignedVirtualPoseMap_rf.find(agentnumber) != mAgentsAssignedVirtualPoseMap_rf.end())
            {
                tmp[agentnumber] = mAgentsAssignedVirtualPoseMap_rf[agentnumber];
            }
        }
        // assignedVirtualPoseMap = mAgentsAssignedVirtualPoseMap_rf;
        assignedVirtualPoseMap = tmp;
    }
    else
    {
        // assignedVirtualPoseMap.clear();
        assignedVirtualPoseMap = mAgentsAssignedVirtualPoseMap_rf;
        ROS_WARN("[Teaming Planner %d]: Agents Assigned Virtual Pose map empty", mSourceSegmentId);
        status = false;
    }
    return status;
}

bool TeamingPlanner::getHumanSystemPose_rf(DistributedFormation::Common::Pose& aHumanSystemPose)
{
    bool status = true;

    if (status)
    {
        aHumanSystemPose = mHumanSystemPose_rf;
    }

    return status;
}

bool TeamingPlanner::getOwnUAVSystemPose_rf(DistributedFormation::Common::Pose& aUAVSystemPose)
{
    bool status = true;

    if (status)
    {
        aUAVSystemPose = mSelfSystemPose_rf;
    }

    return status;
}

void TeamingPlanner::clearPhaseAndTimeMap_rf()
{
    mAgentsPhaseAndTimeMap_rf.clear();

    if(mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Agents Phase Time Map cleared", mSourceSegmentId);
    }
}

void TeamingPlanner::clearPoseMap_rf()
{
    mAgentsPoseMap_rf.clear();
    // clearPhaseAndTimeMap_rf(); // also clear
    if(mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Agents Pose Map cleared", mSourceSegmentId);
    }
}

void TeamingPlanner::clearDirectionUtilityMap_rf()
{
    mAgentsDirectionUtilityMap_rf.clear();

    if(mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Agents Direction Utility Map cleared", mSourceSegmentId);
    }
}

void TeamingPlanner::clearConvexRegion2DMap_rf()
{
    mAgentsConvexRegion2DMap_rf.clear();

    if(mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Agents Convex Region 2D Map cleared", mSourceSegmentId);
    }
}

void TeamingPlanner::clearConvexRegion3DMap_rf()
{
    mAgentsConvexRegion3DMap_rf.clear();

    if(mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Agents Convex Region 3D Map cleared", mSourceSegmentId);
    }
}

void TeamingPlanner::clearAssignedVirtualPoseMap_rf()
{
    mAgentsAssignedVirtualPoseMap_rf.clear();

    if(mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Agents AssignedVirtualPose Map cleared", mSourceSegmentId);
    }
}

