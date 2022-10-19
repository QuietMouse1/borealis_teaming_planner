#include "../include/teaming_planner/teaming_planner.h"

// Subscriber Functions
void TeamingPlanner::goalCallback(const mt_msgs::pose::ConstPtr& aGoal)
{
    DistributedFormation::Common::Pose pose;
    pose.position.x = aGoal->position.x;
    pose.position.y = aGoal->position.y;
    pose.position.z = aGoal->position.z;
    pose.headingRad = aGoal->headingRad;
    //ROS_INFO("[Teaming Planner %d]: Goal Message Received\n", mSourceSegmentId);
}

void TeamingPlanner::selfSystemPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& PoseStamped)
{
    geometry_msgs::PoseStamped tmp_pose;
    tmp_pose.header = PoseStamped->header;
    tmp_pose.pose = PoseStamped->pose;
    mSelfSystemPose = tmp_pose;

    Common::Entity::Pose tmp(tmp_pose);
    mSelfSystemPose_rf.position.x = tmp.position.x;
    mSelfSystemPose_rf.position.y = tmp.position.y;
    mSelfSystemPose_rf.position.z = tmp.position.z;
    mSelfSystemPose_rf.headingRad = tmp.yaw;
    mAgentsPoseMap_rf[mSourceSegmentId] = mSelfSystemPose_rf;
    pubPose_rf(mSourceSegmentId, mSelfSystemPose_rf);

    DistributedFormation::Common::Pose aPose;
    mOwnAgentPose_cp.position(0) = tmp.position.x;
    mOwnAgentPose_cp.position(1) = tmp.position.y;
    mOwnAgentPose_cp.position(2) = tmp.position.z;
    mOwnAgentPose_cp.headingRad = tmp.yaw;
    mAgentsPoseMap_cp[mSourceSegmentId] = mOwnAgentPose_cp;
    pubOwnPoseFunc_cp(mSourceSegmentId, mOwnAgentPose_cp);
}

void TeamingPlanner::selfSystemPoseCallbackUWB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& PoseStampedCovar)
{
    geometry_msgs::PoseStamped tmp_pose;
    tmp_pose.header = PoseStampedCovar->header;
    tmp_pose.pose = PoseStampedCovar->pose.pose;
    mSelfSystemPose = tmp_pose;

    Common::Entity::Pose tmp(tmp_pose);
    mSelfSystemPose_rf.position.x = tmp.position.x;
    mSelfSystemPose_rf.position.y = tmp.position.y;
    mSelfSystemPose_rf.position.z = tmp.position.z;
    mSelfSystemPose_rf.headingRad = tmp.yaw;
    mAgentsPoseMap_rf[mSourceSegmentId] = mSelfSystemPose_rf;
    pubPose_rf(mSourceSegmentId, mSelfSystemPose_rf);


    mOwnAgentPose_cp.position(0) = tmp.position.x;
    mOwnAgentPose_cp.position(1) = tmp.position.y;
    mOwnAgentPose_cp.position(2) = tmp.position.z;
    mOwnAgentPose_cp.headingRad = tmp.yaw;
    mAgentsPoseMap_cp[mSourceSegmentId] = mOwnAgentPose_cp;
    pubOwnPoseFunc_cp(mSourceSegmentId, mOwnAgentPose_cp);
}

void TeamingPlanner::systemPointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& aSystemPointCloud)
{
    std::string sourceFrame = "uav" + std::to_string(mSourceSegmentId) + "/os_sensor";
    mSystemPointCloud.header.frame_id = sourceFrame;
    mSystemPointCloud = *aSystemPointCloud;

    if (mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Point Cloud Received\n", mSourceSegmentId);
    }
}

void TeamingPlanner::systemPointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& aSystemPointCloud2)
{
    std::string sourceFrame = "uav" + std::to_string(mSourceSegmentId) + "/os_sensor";
    DistributedFormation::ProcessPointCloud tmpProcessPointCloud;

    sensor_msgs::PointCloud2 tmp;
    sensor_msgs::PointCloud2 tmpTransformed;
    sensor_msgs::PointCloud tmpToPublish;

    tmpProcessPointCloud.ApplyVoxelFilterToPCL2(*aSystemPointCloud2, tmp);    
    tmp.header.stamp = aSystemPointCloud2->header.stamp; 
    tmp.header.frame_id = aSystemPointCloud2->header.frame_id; 
    geometry_msgs::TransformStamped transformStamped;

    try
    {
        transformStamped = mtfBuffer.lookupTransform("odom", sourceFrame,ros::Time(0));

        if (mPointcloudCallbackVerbose)
        {
            // ROS_INFO("Agent ID %d: Transform stamped: ", mSourceSegmentId);
            // std::cout << transformStamped << std::endl;
        }

        tf2::doTransform(tmp, tmpTransformed, transformStamped);
    }
    catch (tf2::TransformException &ex) {
        ROS_WARN("%s", ex.what());
        ros::Duration(0.5).sleep();
    }

    catch (std::length_error e )
    {
        ROS_WARN("End of tf buffer reached! There is more than 10s time desync somewhere in the tree from lidar frame to odom");
        ROS_WARN("%s", e.what());
        ros::Duration(0.5).sleep(); 
        // handle custom exception
    }

    catch (std::length_error e )
    {
        ROS_WARN("End of tf buffer reached! There is more than 10s time desync somewhere in the tree from lidar frame to odom");
        ROS_WARN("%s", e.what());
        ros::Duration(0.5).sleep(); 
        // handle custom exception
    }

    catch (const std::exception& e) {
        ROS_WARN("%s", e.what());
        ros::Duration(0.5).sleep(); 
        // ...
    } 

    catch (const std::string& e) {
        ROS_WARN("%s", e);
        ros::Duration(0.5).sleep(); 
        // ...
    } 

    catch (...) {
        ROS_WARN("This should not happen but ..");
        ros::Duration(0.5).sleep(); 
        // ...
    }

    sensor_msgs::convertPointCloud2ToPointCloud(tmpTransformed, tmpToPublish);
    mSystemPointCloud = tmpToPublish;

    // mVoxelFilterCloudPublisher_rf.publish(tmpToPublish);
}

void TeamingPlanner::activatePlannerCallback(const std_msgs::Bool::ConstPtr& aBoolActivatePlanner)
{
    if (mModuleState == TeamingPlannerConstants::ModuleState::INITILAISING)
    {
        ROS_INFO("Activate planner failed, planner still initialising");
    }
    else
    {
        mBoolActivatePlanner.data = aBoolActivatePlanner->data;
        if(mBoolActivatePlanner.data)
        {
            mModuleState = TeamingPlannerConstants::ModuleState::RUNNING;
        }
        else
        {
            mModuleState = TeamingPlannerConstants::ModuleState::DEACTIVATED;
        }
    }
}

void TeamingPlanner::UAVModeCallback(const std_msgs::String::ConstPtr& aUAVmode)
{
    mUAVMode = aUAVmode->data;
    std::string str1 ("Follow_Me");
    std::string str2 ("Go_There");

    if (str1.compare(aUAVmode->data.c_str()) == 0) // They compare equal
    {
        if (mTask.type != Common::Entity::MTTaskEnum::FOLLOW_ME)
        {
            mTask.type = Common::Entity::MTTaskEnum::FOLLOW_ME;
            clearOtherAgentsData();
            mHistoryOfHumanPoses_rf.clear(); // reset history
        }
    }
    else if (str2.compare(aUAVmode->data.c_str()) == 0)
    {
        if (mTask.type != Common::Entity::MTTaskEnum::GO_THERE)
        {
            mTask.type = Common::Entity::MTTaskEnum::GO_THERE;
            clearOtherAgentsData();
        }
    }
    else 
    {
        mTask.type = Common::Entity::MTTaskEnum::IDLE;
    }
    if (mDebugVerbose)
    {
        ROS_INFO("Agent %i mode: %s activation state:", mSourceSegmentId, mUAVMode.data(), mBoolActivatePlanner.data);
    }

}

void TeamingPlanner::UAVInputPoseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& aInputPose)
{
    if(mTask.type == Common::Entity::MTTaskEnum::FOLLOW_ME)
    {
        // Add this input pose into its own history of human poses
        geometry_msgs::PoseStamped tmp_pose; 
        tmp_pose.header =  aInputPose->header;
        tmp_pose.pose =  aInputPose->pose;

        Common::Entity::Pose tmp(tmp_pose);
        mHumanSystemPose_rf.position.x = tmp.position.x;
        mHumanSystemPose_rf.position.y = tmp.position.y;
        mHumanSystemPose_rf.position.z = tmp.position.z;
        mHumanSystemPose_rf.headingRad = tmp.yaw;

        // while(mHistoryOfHumanPoses_rf.size() > (mPlanningHorizon/mIntervalDistance) - 1)
        // {
        //     mHistoryOfHumanPoses_rf.erase(mHistoryOfHumanPoses_rf.begin());    
        // }

        DistributedFormation::Common::Pose tPose;
        tPose.position.x = tmp.position.x;
        tPose.position.y = tmp.position.y;
        tPose.position.z = tmp.position.z;
        tPose.headingRad = tmp.yaw;

        mHistoryOfHumanPoses_rf.clear(); 
        mHistoryOfHumanPoses_rf.push_back(tPose); // to send only 1 lmao

        // if (checkAndAddHumanSystemPose(mHistoryOfHumanPoses_rf, tPose))
        // {
        //     mHistoryOfHumanPosesReceived = true;
        // }
        
        if (mDebugVerbose)
        {
            ROS_INFO("[Teaming Planner %d]: Follow Me Input pose Received\n", mSourceSegmentId);
        }
    }
    
    if (mTask.type == Common::Entity::MTTaskEnum::GO_THERE)
    {
        std::cout << "Incoming path" << std::endl;
        for (auto pose : mGoTherePath_cp)
        {
            std::cout << "pose x: " << aInputPose->pose.position.x << " pose y: " << aInputPose->pose.position.y << " pose z: " << aInputPose->pose.position.z << std::endl;
        }

        // Assign this input pose as a go there path
        std::vector<DistributedGlobalPathPlanner::Common::Pose> tmp_vec;
        DistributedGlobalPathPlanner::Common::Pose tmp;
        tmp.position(0) = aInputPose->pose.position.x;
        tmp.position(1) = aInputPose->pose.position.y;;
        tmp.position(2) = aInputPose->pose.position.z;
        tmp_vec.push_back(tmp);

        std::cout << "tmp_vec" << std::endl;
        for (auto pose : tmp_vec)
        {
            std::cout << "pose x: " << pose.position(0) << " pose y: " << pose.position(1) << " pose z: " << pose.position(2) << std::endl;
        }

        mGoTherePath_cp = tmp_vec;

        std::cout << "goTherePath to send to phase sync" << std::endl;
        for (auto pose : mGoTherePath_cp)
        {
            std::cout << "pose x: " << pose.position(0) << " pose y: " << pose.position(1) << " pose z: " << pose.position(2) << std::endl;
        }

        if (mDebugVerbose)
        {
            std::cout << "ZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZZ" << std::endl;
            ROS_INFO("[Teaming Planner %d]: New Go there Input pose Received\n", mSourceSegmentId);
            std::cout << "AAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAAA" << std::endl;
        }


    }
}

void TeamingPlanner::numberOfAgentsInTeamCallback(const std_msgs::Int8MultiArray::ConstPtr& aNumberOfAgents)
{
    mAgentsInTeam.data = aNumberOfAgents->data;
    if (mTeamSize != aNumberOfAgents->data.size()) // Check size
    {
        mTeamSize = aNumberOfAgents->data.size();

        ROS_INFO("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!NEW TEAM DETECTED!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
        ROS_INFO("[Teaming Planner %d]: New team detected!, from %d to %d ", mSourceSegmentId, mAgentsInTeamVector.size(), mTeamSize);

        mHistoryOfHumanPoses_rf.clear(); // reset history
        TeamingPlanner::clearAgentNumberTeamVector(); // reset the Agent number vector
        clearOtherAgentsData();

        for (int agentNumber : aNumberOfAgents->data)
        {
            mAgentsInTeamVector.push_back(agentNumber);
            std::cout << "The new agents are " << agentNumber << std::endl;
             
        }
    }
    else // Same size now check the elements
    {
        std::vector<int> aNumberOfAgentsVector;
        for (int agentNumber : aNumberOfAgents->data)
        {
            aNumberOfAgentsVector.push_back(agentNumber);
        }

        sort(aNumberOfAgentsVector.begin(), aNumberOfAgentsVector.end());    
        sort(mAgentsInTeamVector.begin(), mAgentsInTeamVector.end());

        if (aNumberOfAgentsVector != mAgentsInTeamVector)    
        {    
            ROS_INFO("[Teaming Planner %d: New team detected! With different elements ", mSourceSegmentId);
            TeamingPlanner::clearAgentNumberTeamVector(); // reset the vector

            for (int agentNumber : aNumberOfAgents->data)
            {
                mAgentsInTeamVector.push_back(agentNumber);
                std::cout << "The new agents are " << agentNumber << std::endl;
            }
        }
    }

    if (mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: numberOfAgentsInTeamCallback: %d  mTeamSize: %d\n", mSourceSegmentId, aNumberOfAgents->data.size(), mTeamSize);
    }
}

/* Callbacks used by robot formation */
// Store into map.
void TeamingPlanner::systemPoseCallback_rf(const mt_msgs::pose::ConstPtr& aSystemPose)
{
    // if ( std::find(mAgentsInTeamVector.begin(), mAgentsInTeamVector.end(), aSystemPose->sourceSegmentId) != mAgentsInTeamVector.end() )
    // {
        DistributedFormation::Common::Pose tmp;
        tmp.position.x = aSystemPose->position.x;
        tmp.position.y = aSystemPose->position.y;
        tmp.position.z = aSystemPose->position.z;
        tmp.headingRad = aSystemPose->headingRad;

        mAgentsPoseMap_rf[aSystemPose->sourceSegmentId] = tmp;

        if (mAgentPoseCallbackVerbose)
        {
            ROS_INFO("[Teaming Planner %d]: RF System Pose Received from Agent: %d\n", mSourceSegmentId, aSystemPose->sourceSegmentId);
            for (auto agent : mAgentsInTeamVector)
            {
                ROS_INFO("Agent number in team : %d", agent);
            }
        }
    // }

}

void TeamingPlanner::phaseTimeCallback_rf(const mt_msgs::phaseAndTime::ConstPtr& aPhaseAndTime)
{    
    // if ( std::find(mAgentsInTeamVector.begin(), mAgentsInTeamVector.end(), aPhaseAndTime->sourceSegmentId) != mAgentsInTeamVector.end() )
    // {
        DistributedFormation::Common::PhaseAndTime tmp;
        tmp.phase = static_cast<DistributedFormation::Common::PHASE>(aPhaseAndTime->phase);
        tmp.timeMicroSecs = aPhaseAndTime->time;
        mAgentsPhaseAndTimeMap_rf[aPhaseAndTime->sourceSegmentId] = tmp;
        if (mPhaseSyncCallbackVerbose)
        {
            ROS_INFO("[Teaming Planner %d]: Phase Time Received from Agent: %d\n", mSourceSegmentId, aPhaseAndTime->sourceSegmentId);
        }
    // }
}

void TeamingPlanner::directionUtilityCallback_rf(const mt_msgs::angleIndexAndUtility::ConstPtr& aDirectionUtility)
{
    // if ( std::find(mAgentsInTeamVector.begin(), mAgentsInTeamVector.end(), aDirectionUtility->sourceSegmentId) != mAgentsInTeamVector.end() )
    // {
// 
        DistributedFormation::Common::DirectionUtility tmp;

        tmp.angleIndexAndUtility = aDirectionUtility->angleIndexAndUtility;
        mAgentsDirectionUtilityMap_rf[aDirectionUtility->sourceSegmentId] = tmp;

        if (mDebugVerbose)
        {
            ROS_INFO("[Teaming Planner %d]: Direction Utility Received from Agent: %d\n", mSourceSegmentId, aDirectionUtility->sourceSegmentId);
        }
    // }
}

void TeamingPlanner::convexRegion2DCallback_rf(const mt_msgs::convexRegion2D::ConstPtr& aConvexRegion2D)
{
    
    // if ( std::find(mAgentsInTeamVector.begin(), mAgentsInTeamVector.end(), aConvexRegion2D->sourceSegmentId) != mAgentsInTeamVector.end() )
    // {

        DistributedFormation::Common::ConvexRegion2D tmp;

        std::vector<double> vecACol1 = aConvexRegion2D->matrixACol1;
        std::vector<double> vecACol2 = aConvexRegion2D->matrixACol2;
        std::vector<double> vecB = aConvexRegion2D->matrixB;

        tmp.A.conservativeResize(vecACol1.size(), Eigen::NoChange);
        tmp.b.conservativeResize(vecB.size(), Eigen::NoChange);

        tmp.A.col(0) = Eigen::Map<Eigen::VectorXd>(&vecACol1[0], vecACol1.size());
        tmp.A.col(1) = Eigen::Map<Eigen::VectorXd>(&vecACol2[0], vecACol2.size());
        tmp.b = Eigen::Map<Eigen::VectorXd>(&vecB[0], vecB.size());

        mAgentsConvexRegion2DMap_rf[aConvexRegion2D->sourceSegmentId] = tmp;

        if (mDebugVerbose)
        {
            ROS_INFO("[Teaming Planner %d]: Convex Region Received from Agent: %d\n", mSourceSegmentId, aConvexRegion2D->sourceSegmentId);
        }

        // if (mDebugVerbose)
        // {
        //     ROS_INFO("convexRegion2DCallback_rf Vector A column 1 Values: ");
        //     for (auto value : vecACol1)
        //     {
        //         ROS_INFO("%d ", value);
        //     }
        //     ROS_INFO("\n");

        //     ROS_INFO("convexRegion2DCallback_rf Vector A column 2 Values: ");
        //     for (auto value : vecACol2)
        //     {
        //         ROS_INFO("%d ", value);
        //     }
        //     ROS_INFO("\n");

        //     ROS_INFO("convexRegion2DCallback_rf Vector B Values: ");
        //     for (auto value : vecB)
        //     {
        //         ROS_INFO("%d ", value);
        //     }
        //     ROS_INFO("\n");
        // }
    // }
}

void TeamingPlanner::convexRegion3DCallback_rf(const mt_msgs::convexRegion3D::ConstPtr& aConvexRegion3D)
{
    // if ( std::find(mAgentsInTeamVector.begin(), mAgentsInTeamVector.end(), aConvexRegion3D->sourceSegmentId) != mAgentsInTeamVector.end() )
    // {

        DistributedFormation::Common::ConvexRegion3D tmp;

        std::vector<double> vecACol1 = aConvexRegion3D->matrixACol1;
        std::vector<double> vecACol2 = aConvexRegion3D->matrixACol2;
        std::vector<double> vecACol3 = aConvexRegion3D->matrixACol3;
        std::vector<double> vecB = aConvexRegion3D->matrixB;

        tmp.A.conservativeResize(vecACol1.size(), Eigen::NoChange);
        tmp.b.conservativeResize(vecB.size(), Eigen::NoChange);

        tmp.A.col(0) = Eigen::Map<Eigen::VectorXd>(&vecACol1[0], vecACol1.size());
        tmp.A.col(1) = Eigen::Map<Eigen::VectorXd>(&vecACol2[0], vecACol2.size());
        tmp.A.col(2) = Eigen::Map<Eigen::VectorXd>(&vecACol3[0], vecACol3.size());
        tmp.b = Eigen::Map<Eigen::VectorXd>(&vecB[0], vecB.size());

        mAgentsConvexRegion3DMap_rf[aConvexRegion3D->sourceSegmentId] = tmp;

        if (mDebugVerbose)
        {
            ROS_INFO("[Teaming Planner %d]: Convex Region Received from Agent: %d", mSourceSegmentId, aConvexRegion3D->sourceSegmentId);
        }

        // if (mDebugVerbose)
        // {
        //     ROS_INFO("convexRegion3DCallback_rf Vector A column 1 Values: ");
        //     for (auto value : vecACol1)
        //     {
        //         ROS_INFO("%d ", value);
        //     }
        //     ROS_INFO("\n");

        //     ROS_INFO("convexRegion3DCallback_rf Vector A column 2 Values: ");
        //     for (auto value : vecACol2)
        //     {
        //         ROS_INFO("%d ", value);
        //     }
        //     ROS_INFO("\n");

        //     ROS_INFO("convexRegion3DCallback_rf Vector A column 3 Values: ");
        //     for (auto value : vecACol3)
        //     {
        //         ROS_INFO("%d ", value);
        //     }
        //     ROS_INFO("\n");

        //     ROS_INFO("convexRegion3DCallback_rf Vector B Values: ");
        //     for (auto value : vecB)
        //     {
        //         ROS_INFO("%d ", value);
        //     }
        //     ROS_INFO("\n");
        // }
    // }
}

void TeamingPlanner::assignedVirtualPoseMapCallback_rf(const mt_msgs::posevector::ConstPtr& aAssignedVirtualPoseMap)
{    
    // if ( std::find(mAgentsInTeamVector.begin(), mAgentsInTeamVector.end(), aAssignedVirtualPoseMap->sourceSegmentId) != mAgentsInTeamVector.end() )
    // {

        std::unordered_map<int32_t, DistributedFormation::Common::Pose> tmpMap;
        for (auto pose : aAssignedVirtualPoseMap->poseVector)
        {
            DistributedFormation::Common::Pose tmp;
            tmp.position.x = pose.position.x;
            tmp.position.y = pose.position.y;
            tmp.position.z = pose.position.z;

            tmp.headingRad = pose.headingRad;

            
            tmpMap[pose.sourceSegmentId] = tmp;
        }
        mAgentsAssignedVirtualPoseMap_rf[aAssignedVirtualPoseMap->sourceSegmentId] = tmpMap;

        if(mDebugVerbose)
        {
            ROS_INFO("Assigned Virtual Position Received from Agent: %d", aAssignedVirtualPoseMap->sourceSegmentId);
        }
    // }
}

/* Global Consensus path callbacks*/
void TeamingPlanner::phaseTimeCallback_cp(const mt_msgs::phaseAndTime::ConstPtr& aPhaseAndTime)
{    
    // if ( std::find(mAgentsInTeamVector.begin(), mAgentsInTeamVector.end(), aPhaseAndTime->sourceSegmentId) != mAgentsInTeamVector.end() )
    // {
        DistributedGlobalPathPlanner::Common::PhaseAndTime tmp;
        tmp.phase = static_cast<DistributedGlobalPathPlanner::Common::PHASE>(aPhaseAndTime->phase);
        tmp.timeMicroSecs = aPhaseAndTime->time;
        mAgentsPhasesAndTimeMap_cp[aPhaseAndTime->sourceSegmentId] = tmp;

        if (mPhaseSyncCallbackVerbose)
        {
            ROS_INFO("[Teaming Planner CP %d]: Phase Time Received from Agent: %d", mSourceSegmentId, aPhaseAndTime->sourceSegmentId);
        }
    // }
}

void TeamingPlanner::systemPoseCallback_cp(const mt_msgs::pose::ConstPtr& aSystemPose)
{    
    DistributedGlobalPathPlanner::Common::Pose tmp;
    tmp.position(0) = aSystemPose->position.x;
    tmp.position(1) = aSystemPose->position.y;
    tmp.position(2) = aSystemPose->position.z;
    tmp.headingRad = aSystemPose->headingRad;

    mAgentsPoseMap_cp[aSystemPose->sourceSegmentId] = tmp;

    if (mSystemPoseCallbackVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: CP System Pose Received from Agent: %d", mSourceSegmentId, aSystemPose->sourceSegmentId);
    }
}

void TeamingPlanner::pathAndProgressCallback_cp(const mt_msgs::pathAndProgress::ConstPtr& aPathAndProgress)
{    
    DistributedGlobalPathPlanner::Common::PathAndWaypointProgress agents_path_progress_tmp;
    std::vector<DistributedGlobalPathPlanner::Common::Pose> tmp_poses;
    for (auto pose : aPathAndProgress->poseVector)
    {
        DistributedGlobalPathPlanner::Common::Pose tmp_pose;
        tmp_pose.position(0) = pose.position.x;
        tmp_pose.position(1) = pose.position.y;
        tmp_pose.position(2)= pose.position.z;
        tmp_poses.push_back(tmp_pose);
    }

    agents_path_progress_tmp.poses = tmp_poses;
    agents_path_progress_tmp.waypointProgress = aPathAndProgress->waypointProgress;
    mAgentsPathAndWaypointProgressMap_cp[aPathAndProgress->sourceSegmentId] = agents_path_progress_tmp;
    if (mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Path and progress recieved! from %i", mSourceSegmentId, aPathAndProgress->sourceSegmentId);
    }
}

void TeamingPlanner::plannedPathCallback_cp(const mt_msgs::posevector::ConstPtr& aPlannedPath)
{    
    std::vector<Eigen::Vector3d> tmp_poses;
    for (auto pose : aPlannedPath->poseVector)
    {
        Eigen::Vector3d pose_eigen;
        pose_eigen(0) = pose.position.x;
        pose_eigen(1) = pose.position.y;
        pose_eigen(2) = pose.position.z;
        tmp_poses.push_back(pose_eigen);
    }
    mAgentsPlannedPathMap_cp[aPlannedPath->sourceSegmentId] = tmp_poses;
    if (mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: Planned path recieved! from %i", mSourceSegmentId, aPlannedPath->sourceSegmentId);
    }
}

void TeamingPlanner::agentProcessedPathOfAgentsCallback_cp(const mt_msgs::pathAndCostVector::ConstPtr& aPathAndCostVector)
{    
    std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PathAndCost> path_and_cost_tmp_map;
    for (auto path_cost : aPathAndCostVector->pathAndCostVector)
    {
        DistributedGlobalPathPlanner::Common::PathAndCost path_and_cost_tmp;
        std::vector<Eigen::Vector3d> tmp_poses;
        for (auto pose : path_cost.poseVector)
        {            
            Eigen::Vector3d pose_eigen;
            pose_eigen(0) = pose.position.x;
            pose_eigen(1) = pose.position.y;
            pose_eigen(2) = pose.position.z;
            tmp_poses.push_back(pose_eigen);
        }
        path_and_cost_tmp.positions = tmp_poses;
        path_and_cost_tmp.cost = path_cost.cost;
        path_and_cost_tmp_map[path_cost.sourceSegmentId] =  path_and_cost_tmp;
    }
    mAgentsProcessedPathOfAgentsMap_cp[aPathAndCostVector->sourceSegmentId] = path_and_cost_tmp_map;

    if (mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: aPathAndCostVector recieved! from %i", mSourceSegmentId, aPathAndCostVector->sourceSegmentId);
    }
}

void TeamingPlanner::agentBestProcessedPathCallback_cp(const mt_msgs::posevector::ConstPtr& aBestProcessedPath)
{    
    std::vector<Eigen::Vector3d> tmp_poses;
    for (auto pose : aBestProcessedPath->poseVector)
    {
        Eigen::Vector3d pose_eigen;
        pose_eigen(0) = pose.position.x;
        pose_eigen(1) = pose.position.y;
        pose_eigen(2) = pose.position.z;
        tmp_poses.push_back(pose_eigen);
    }
    mAgentsBestProcessedPath_cp[aBestProcessedPath->sourceSegmentId] = tmp_poses;

    if (mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: aBestProcessedPath recieved! from %i", mSourceSegmentId, aBestProcessedPath->sourceSegmentId);
    }
}

void TeamingPlanner::ProcessedGoTherePathCallback(const geometry_msgs::PoseArray::ConstPtr& aInputPoseArray)
{
    std::vector<DistributedFormation::Common::Pose> tmp_vec;

    for (auto pose : aInputPoseArray->poses)
    {
        DistributedFormation::Common::Pose tmp_pose;
        tmp_pose.position.x = pose.position.x;
        tmp_pose.position.y = pose.position.y;
        tmp_pose.position.z = pose.position.z;
        tmp_pose.headingRad = 0;
        tmp_vec.push_back(tmp_pose);
    }

    if (mDebugVerbose)
    {
        ROS_INFO("[Teaming Planner %d]: ProcessedGoTherePathCallback recieved! ", mSourceSegmentId);
    }

    mProcessedGoTherePath = tmp_vec;
}

void TeamingPlanner::teamingAgentRadiusCallback(const std_msgs::Float32::ConstPtr& aAgentRadius)
{
    mRobotFormationParameters.agentRadius = aAgentRadius->data;
    mDistributedFormation.SetParameters(mRobotFormationParameters);
    
}

void TeamingPlanner::clearAgentNumberTeamVector()
{
    mAgentsInTeamVector.clear();
    ROS_INFO("[Teaming Planner %d]: Agents in Team Vector info cleared!", mSourceSegmentId);
}


// helper funcs

void TeamingPlanner::clearOtherAgentsData()
{
    clearPhaseAndTimeMap_rf(); 
    clearPoseMap_rf(); 
    clearDirectionUtilityMap_rf();
    clearConvexRegion2DMap_rf();
    clearConvexRegion3DMap_rf();
    clearAssignedVirtualPoseMap_rf(); 

    clearPhasesAndTime_cp();
    clearAgentsPoseBuffer_cp();
    clearAgentsProcessedPathOfAgentsBuffer_cp();
    clearAgentsPlannedPathBuffer_cp();
    clearAgentsPathAndWaypointProgressBuffer_cp();
    clearAgentsBestProcessedPathBuffer_cp();
    ROS_INFO("[Teaming Planner %d]: Other Agents info cleared!", mSourceSegmentId);
}

void TeamingPlanner::resetData()
{
    mHistoryOfHumanPoses_rf.clear(); // reset history
    TeamingPlanner::clearAgentNumberTeamVector(); // reset the Agent number vector
    clearOtherAgentsData(); // Clear map about other agents
    // mDistributedFormation.ResetAndClearTeamAgentsPoseAndPhases(); // Clear buffer in the formation planner
    // mGlobalPathPlanner.ResetAndClearTeamAgentsPoseAndPhases(); // Clear buffer in the path planner
}


void TeamingPlanner::paramServerCallback(borealis_teaming_planner::borealis_teaming_planner_Config &config, uint32_t level)
{
    std::cout << "Reconfigure request" << std::endl;
    std::cout << "agent_radius: " << config.agent_radius << "\tpriority_penalty: " << config.priority_penalty << std::endl;
    std::cout << "triangle_size: " << config.triangle_size << "\tline_size: " << config.line_size << std::endl;
    std::cout << "triangle_size: " << config.point_removal_radius << std::endl;

    mRobotFormationParameters.agentRadius = config.agent_radius;
    mRobotFormationParameters.priorityPenalty = config.priority_penalty;
    mRobotFormationParameters.desiredDistanceInTriFormation = config.triangle_size;
    mRobotFormationParameters.desiredDistanceInLineFormation = config.line_size;
    mRobotFormationParameters.pointRemovalRadius = config.point_removal_radius;
    mDistributedFormation.SetParameters(mRobotFormationParameters);

}
