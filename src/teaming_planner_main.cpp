#include "../include/teaming_planner/teaming_planner.h"
#include "../../../distributed_multi_robot_formation/src/DistributedMultiRobotFormationHandler.h"
#include "../../../distributed_multi_robot_formation/src/DistributedMultiRobotFormation.h"

void TeamingPlanner::teamingPlannerMain()
{
    switch (mModuleState)
    {
    case TeamingPlannerConstants::ModuleState::INITILAISING:
        if (!mModuleStateVerbose)
        {
            ROS_INFO("Module State Initialising\n");
            mModuleStateVerbose = true;
        }

        // attach distributed path planner handler
        mHistoryOfHumanPoses_rf.reserve(mPlanningHorizon / mIntervalDistance);

        mHandlerPtr->m_getNumberOfAgentsInTeam = std::bind(&TeamingPlanner::getNumberOfAgentsInTeam, this, std::placeholders::_1);
        mHandlerPtr->m_getOwnAgentID = std::bind(&TeamingPlanner::getOwnAgentId, this, std::placeholders::_1);
        mHandlerPtr->m_getOwnAgentLidarPointCloud = std::bind(&TeamingPlanner::getOwnAgentLidarPointCloud, this, std::placeholders::_1);
        mHandlerPtr->m_getPosesForFormationToTrack = std::bind(&TeamingPlanner::getPosesForFormationToTrack_rf, this, std::placeholders::_1); // HIstory of human poses

        mHandlerPtr->m_getPhasesAndTimeRecordOfAgents = std::bind(&TeamingPlanner::getPhaseAndTimeMap_rf, this, std::placeholders::_1);
        mHandlerPtr->m_pubOwnPhaseAndTime = std::bind(&TeamingPlanner::pubPhaseAndTime_rf, this, std::placeholders::_1, std::placeholders::_2);
        mHandlerPtr->m_clearAgentsPoseBuffer = std::bind(&TeamingPlanner::clearPoseMap_rf, this);
        mHandlerPtr->m_clearPhasesAndTimeRecordOfAgentsBuffer = std::bind(&TeamingPlanner::clearPhaseAndTimeMap_rf, this);
        mHandlerPtr->m_pubOwnPoseFunc = std::bind(&TeamingPlanner::pubPose_rf, this, std::placeholders::_1, std::placeholders::_2);
        mHandlerPtr->m_getOwnAgentPose = std::bind(&TeamingPlanner::getOwnUAVSystemPose_rf, this, std::placeholders::_1);
        mHandlerPtr->m_getAgentsPose = std::bind(&TeamingPlanner::getPoseMap_rf, this, std::placeholders::_1);
        mHandlerPtr->m_getHumanPose = std::bind(&TeamingPlanner::getHumanSystemPose_rf, this, std::placeholders::_1);
        mHandlerPtr->m_clearAgentsDirectionUtilityBuffer = std::bind(&TeamingPlanner::clearDirectionUtilityMap_rf, this);
        mHandlerPtr->m_pubOwnDirectionUtility = std::bind(&TeamingPlanner::pubDirectionUtility_rf, this, std::placeholders::_1, std::placeholders::_2);
        mHandlerPtr->m_getAgentsDirectionUtility = std::bind(&TeamingPlanner::getDirectionUtilityMap_rf, this, std::placeholders::_1);
        mHandlerPtr->m_clearAgentsConvexRegion2DBuffer = std::bind(&TeamingPlanner::clearConvexRegion2DMap_rf, this);
        mHandlerPtr->m_pubOwnConvex2DRegion = std::bind(&TeamingPlanner::pubConvexRegion2D_rf, this, std::placeholders::_1, std::placeholders::_2);
        mHandlerPtr->m_getAgentsConvex2DRegion = std::bind(&TeamingPlanner::getConvexRegion2DMap_rf, this, std::placeholders::_1);
        mHandlerPtr->m_clearAgentsConvexRegion3DBuffer = std::bind(&TeamingPlanner::clearConvexRegion3DMap_rf, this);
        mHandlerPtr->m_pubOwnConvex3DRegion = std::bind(&TeamingPlanner::pubConvexRegion3D_rf, this, std::placeholders::_1, std::placeholders::_2);
        mHandlerPtr->m_getAgentsConvex3DRegion = std::bind(&TeamingPlanner::getConvexRegion3DMap_rf, this, std::placeholders::_1);
        mHandlerPtr->m_pubOwnTaskAssignments = std::bind(&TeamingPlanner::pubAssignedPoseMap_rf, this, std::placeholders::_1, std::placeholders::_2);
        mHandlerPtr->m_clearAgentsTaskAssignmentsBuffer = std::bind(&TeamingPlanner::clearPoseMap_rf, this); // huh
        mHandlerPtr->m_getAgentsTaskAssignments = std::bind(&TeamingPlanner::getAssignedVirtualPoseMap_rf, this, std::placeholders::_1);
        mHandlerPtr->m_pubOwnAgentAssignedPose = std::bind(&TeamingPlanner::pubAssignedPose_rf, this, std::placeholders::_1, std::placeholders::_2);

        mDistributedFormation.AttachHandler(mHandlerPtr);
        ROS_INFO("Finished binding Distributed formation handler");

        // attach consensus path
        ROS_INFO("Binding consensus path planner handler");
        mGlobalPathPlannerHandlerPtr->m_getNumberOfAgentsInTeam = std::bind(&TeamingPlanner::getNumberOfAgentsInTeam, this, std::placeholders::_1);
        mGlobalPathPlannerHandlerPtr->m_getOwnAgentID = std::bind(&TeamingPlanner::getOwnAgentId, this, std::placeholders::_1);
        mGlobalPathPlannerHandlerPtr->m_getOwnAgentLidarPointCloud = std::bind(&TeamingPlanner::getOwnAgentLidarPointCloud, this, std::placeholders::_1);
        mGlobalPathPlannerHandlerPtr->m_getGoTherePath = std::bind(&TeamingPlanner::getGoTherePath_cp, this, std::placeholders::_1);
        mGlobalPathPlannerHandlerPtr->m_getPhasesAndTimeRecordOfAgents = std::bind(&TeamingPlanner::getPhasesAndTimeRecordOfAgents_cp, this, std::placeholders::_1);
        mGlobalPathPlannerHandlerPtr->m_pubOwnPhaseAndTime = std::bind(&TeamingPlanner::pubOwnPhaseAndTime_cp, this, std::placeholders::_1, std::placeholders::_2);
        mGlobalPathPlannerHandlerPtr->m_clearAgentsPoseBuffer = std::bind(&TeamingPlanner::clearAgentsPoseBuffer_cp, this);
        mGlobalPathPlannerHandlerPtr->m_clearPhasesAndTimeRecordOfAgentsBuffer = std::bind(&TeamingPlanner::clearPhasesAndTime_cp, this);

        mGlobalPathPlannerHandlerPtr->m_pubOwnPoseFunc = std::bind(&TeamingPlanner::pubOwnPoseFunc_cp, this, std::placeholders::_1, std::placeholders::_2);
        mGlobalPathPlannerHandlerPtr->m_getOwnAgentPose = std::bind(&TeamingPlanner::getOwnAgentPose_cp, this, std::placeholders::_1);
        mGlobalPathPlannerHandlerPtr->m_getAgentsPose = std::bind(&TeamingPlanner::getAgentsPose_cp, this, std::placeholders::_1);
        mGlobalPathPlannerHandlerPtr->m_clearAgentsPathAndWaypointProgressBuffer = std::bind(&TeamingPlanner::clearAgentsPathAndWaypointProgressBuffer_cp, this);
        mGlobalPathPlannerHandlerPtr->m_getAgentsPathAndWaypointProgress = std::bind(&TeamingPlanner::getAgentsPathAndWaypointProgress_cp, this, std::placeholders::_1);
        mGlobalPathPlannerHandlerPtr->m_pubOwnPathAndWaypointProgress = std::bind(&TeamingPlanner::pubOwnPathAndWaypointProgress_cp, this, std::placeholders::_1, std::placeholders::_2);
        mGlobalPathPlannerHandlerPtr->m_clearAgentsPlannedPathBuffer = std::bind(&TeamingPlanner::clearAgentsPlannedPathBuffer_cp, this);
        mGlobalPathPlannerHandlerPtr->m_getAgentsPlannedPath = std::bind(&TeamingPlanner::getAgentsPlannedPath_cp, this, std::placeholders::_1);
        mGlobalPathPlannerHandlerPtr->m_pubOwnPlannedPath = std::bind(&TeamingPlanner::pubOwnPlannedPath_cp, this, std::placeholders::_1, std::placeholders::_2);
        mGlobalPathPlannerHandlerPtr->m_clearAgentsProcessedPathOfAgentsBuffer = std::bind(&TeamingPlanner::clearAgentsProcessedPathOfAgentsBuffer_cp, this);
        mGlobalPathPlannerHandlerPtr->m_getAgentsProcessedPathOfAgents = std::bind(&TeamingPlanner::getAgentsProcessedPathOfAgents_cp, this, std::placeholders::_1);
        mGlobalPathPlannerHandlerPtr->m_pubOwnProcessedPathOfAgents = std::bind(&TeamingPlanner::pubOwnProcessedPathOfAgents_cp, this, std::placeholders::_1, std::placeholders::_2);
        mGlobalPathPlannerHandlerPtr->m_clearAgentsBestProcessedPathBuffer = std::bind(&TeamingPlanner::clearAgentsBestProcessedPathBuffer_cp, this);
        mGlobalPathPlannerHandlerPtr->m_getAgentsBestProcessedPath = std::bind(&TeamingPlanner::getAgentsBestProcessedPath_cp, this, std::placeholders::_1);
        mGlobalPathPlannerHandlerPtr->m_pubOwnBestProcessedPath = std::bind(&TeamingPlanner::pubOwnBestProcessedPath_cp, this, std::placeholders::_1, std::placeholders::_2);
        mGlobalPathPlannerHandlerPtr->m_pubProcessedGoTherePath = std::bind(&TeamingPlanner::pubProcessedGoTherePath_cp, this, std::placeholders::_1, std::placeholders::_2);

        ROS_INFO("Attaching path planner handler ");
        mGlobalPathPlanner.AttachHandler(mGlobalPathPlannerHandlerPtr);

        // ROS_INFO("Attaching path planner ptr handler ");
        // mGlobalPathPlannerPtr->AttachHandler(mGlobalPathPlannerHandlerPtr);
        ROS_INFO("Finished binding global path planner ");
        // Set parameters
        ROS_INFO("Reading and setting parameters");
        readParameters();
        mDistributedFormation.SetParameters(mRobotFormationParameters);
        mGlobalPathPlanner.SetParameters(mGlobalPathPlanParameters, mPathPlanningParameters);

        // mGlobalPathPlannerPtr->SetParameters(mGlobalPathPlanParameters, mPathPlanningParameters);

        ROS_INFO("Finished reading and setting parameters");

        // Finished init
        mModuleState = TeamingPlannerConstants::ModuleState::RUNNING;
        mTask.type = Common::Entity::MTTaskEnum::IDLE;
        ROS_INFO("Finished Init");
        
        break;

    case TeamingPlannerConstants::ModuleState::READY:

        if (!mModuleStateVerbose)
        {
            ROS_INFO("Module State Ready, waiting for Task Command\n");
            mModuleStateVerbose = true;
        }

        break;

    case TeamingPlannerConstants::ModuleState::RUNNING:
        if (!mModuleStateVerbose)
        {
            ROS_INFO("Module State Running\n");
            // printOutDroneMapVariables();
            mModuleStateVerbose = true;
        }
        std::cout << "/////////////////////" << std::endl;
        std::cout << "Team size is " << mTeamSize << std::endl;
        for (auto agent_number: mAgentsInTeamVector)
        {
            std::cout << "The team contains " << agent_number << std::endl;
        }
        std::cout << "/////////////////////" << std::endl;

        switch (mTask.type)
        {
            case Common::Entity::MTTaskEnum::FOLLOW_ME:
            {
                if (!mModuleTaskVerbose)
                {
                    ROS_INFO("[Teaming Planner %d]: Follow Me Generating Formation\n", mSourceSegmentId);
                    // printOutDroneMapVariables();
                    mModuleTaskVerbose = true;
                }
                mDistributedFormation.RunDistributedFormation();
                break;
            }

            case Common::Entity::MTTaskEnum::GO_THERE:
            {
                if (!mModuleTaskVerbose)
                {
                    // ROS_INFO("Go There\n");
                    mModuleTaskVerbose = true;
                }
                // printOutDroneMapVariables();
                ROS_INFO("[Teaming Planner %d]: Go there! Generating Path", mSourceSegmentId);
                // mGlobalPathPlannerPtr->RunDistributedGlobalPathPlanner();
                mGlobalPathPlanner.RunDistributedGlobalPathPlanner();
                ROS_INFO("[Teaming Planner %d]: Go there! Generating Formation", mSourceSegmentId);
                mDistributedFormation.RunDistributedFormation();
                break;
            }

            case Common::Entity::MTTaskEnum::IDLE:
            {
                ROS_INFO("Teaming Planner %d]: PLANNER IDLING!", mSourceSegmentId);
                if (!mModuleTaskVerbose)
                {
                    // ROS_INFO("Go There\n");
                    mModuleTaskVerbose = true;
                }
                break;
            }
        }
        break;
    case TeamingPlannerConstants::ModuleState::DEACTIVATED:
        if (!mModuleStateVerbose)
        {
            ROS_INFO("Module State Deactivated\n");
            mModuleStateVerbose = true;
        }
        break;
    default:
        if (!mModuleStateVerbose)
        {
            // ROS_INFO("Module State Not Applicable\n");
            mModuleStateVerbose = true;
        }
        break;
    }
}

bool TeamingPlanner::checkAndAddHumanSystemPose(std::vector<DistributedFormation::Common::Pose> &historyOfHumanPoses, const DistributedFormation::Common::Pose aPose)
{
    bool status(false);
    if (historyOfHumanPoses.empty())
    {
        historyOfHumanPoses.emplace_back(aPose);
        status = true;
    }
    else
    {
        double tDist = euclideanDistance(historyOfHumanPoses.back().position.x, historyOfHumanPoses.back().position.y, aPose.position.x, aPose.position.y);
        if (tDist >= mIntervalDistance)
        {
            historyOfHumanPoses.emplace_back(aPose);
            status = true;
        }
        else
        {
            status = false;
        }
    }
    return status;
}

double TeamingPlanner::euclideanDistance(const double x1, const double y1, const double x2, const double y2)
{
    double x = (x2 - x1) * (x2 - x1);
    double y = (y2 - y1) * (y2 - y1);
    return std::sqrt(x + y);
}

void TeamingPlanner::readParameters()
{
    // DistributedFormationParameters()
    // : workspace(Common::WORKSPACE::DIM_3_WITH_ROT)
    // , expiryDurationMicroSec(15*1000000)
    // , numberOfAzimuthDiscreteAnglesOnASide(0)
    // , resolutionAzimuthAngleRad(0.0)
    // , numberOfElevationDiscreteAnglesOnASide(0)
    // , resolutionElevationAngleRad(0.0)
    // , distanceToFollowBehind(-1.0)
    // , localBoundingBoxForPathAlongX(3.0)
    // , localBoundingBoxForPathAlongY(3.0)
    // , localBoundingBoxForPathAlongZ(2.0)
    // , pointRemovalRadius(0.45)
    // , desiredDistanceInTriFormation(2.5)
    // , desiredDistanceInLineFormation(1.25)
    // , incrementOffsetToFormationYaw(M_PI / 2.0)
    // , agentRadius(0.3)
    // , waypointReachedBoundary(1.5)
    // , weightForGoal(0.3)
    // , weightForRotation(0.3)
    // , weightForSize(0.4)
    // , desiredHeight(2.0)
    // , priorityPenalty(1.0)
    // {}

    // robot formation parameters
    mRobotFormationParameters.workspace = DistributedFormation::Common::WORKSPACE::DIM_2_WITH_YAW; // hmm
    mRobotFormationParameters.expiryDurationMicroSec = 15 * 1000000;
    mConfigFileReader.getParam(mNhPrivate, "numberOfAzimuthDiscreteAnglesOnASide", mRobotFormationParameters.numberOfAzimuthDiscreteAnglesOnASide, 0);
    mConfigFileReader.getParam(mNhPrivate, "resolutionAzimuthAngleRad", mRobotFormationParameters.resolutionAzimuthAngleRad, 0.0);
    mConfigFileReader.getParam(mNhPrivate, "numberOfElevationDiscreteAnglesOnASide", mRobotFormationParameters.numberOfElevationDiscreteAnglesOnASide, 0);
    mConfigFileReader.getParam(mNhPrivate, "resolutionElevationAngleRad", mRobotFormationParameters.resolutionElevationAngleRad, 0.0);
    mConfigFileReader.getParam(mNhPrivate, "distanceToFollowBehind", mRobotFormationParameters.distanceToFollowBehind, -1.0);
    mConfigFileReader.getParam(mNhPrivate, "localBoundingBoxForPathAlongX", mRobotFormationParameters.localBoundingBoxForPathAlongX, 3.0);
    mConfigFileReader.getParam(mNhPrivate, "localBoundingBoxForPathAlongY", mRobotFormationParameters.localBoundingBoxForPathAlongY, 3.0);
    mConfigFileReader.getParam(mNhPrivate, "localBoundingBoxForPathAlongZ", mRobotFormationParameters.localBoundingBoxForPathAlongZ, 2.0);
    mConfigFileReader.getParam(mNhPrivate, "pointRemovalRadius", mRobotFormationParameters.pointRemovalRadius, 0.45);
    mConfigFileReader.getParam(mNhPrivate, "desiredDistanceInTriFormation", mRobotFormationParameters.desiredDistanceInTriFormation, 2.5);
    mConfigFileReader.getParam(mNhPrivate, "desiredDistanceInLineFormation", mRobotFormationParameters.desiredDistanceInLineFormation, 1.25);
    mConfigFileReader.getParam(mNhPrivate, "incrementOffsetToFormationYaw", mRobotFormationParameters.incrementOffsetToFormationYaw, 0.0);
    mConfigFileReader.getParam(mNhPrivate, "agentRadius", mRobotFormationParameters.agentRadius, 0.3);
    mConfigFileReader.getParam(mNhPrivate, "waypointReachedBoundary", mRobotFormationParameters.waypointReachedBoundary, 1.5);
    mConfigFileReader.getParam(mNhPrivate, "weightForGoal", mRobotFormationParameters.weightForGoal, 0.3);
    mConfigFileReader.getParam(mNhPrivate, "weightForRotation", mRobotFormationParameters.weightForRotation, 0.40);
    mConfigFileReader.getParam(mNhPrivate, "weightForSize", mRobotFormationParameters.weightForSize, 0.40);
    mConfigFileReader.getParam(mNhPrivate, "desiredHeight", mRobotFormationParameters.desiredHeight, 2.0);
    mConfigFileReader.getParam(mNhPrivate, "priorityPenalty", mRobotFormationParameters.priorityPenalty, 1.0);

    // Common::DIMENSION dimension;
    // int64_t expiryDurationMicroSec;
    // double pointRemovalRadius;
    // double agentRadius;
    // double waypointReachedBoundary;
    // double desiredHeight;

    // DistributedGlobalPathParams()
    // : dimension(Common::DIMENSION::DIM_2)
    // , expiryDurationMicroSec(15*1000000)
    // , pointRemovalRadius(0.45)
    // , agentRadius(0.3)
    // , waypointReachedBoundary(1.5)
    // , desiredHeight(2.0)

    mGlobalPathPlanParameters.dimension = DistributedGlobalPathPlanner::Common::DIMENSION::DIM_2;
    mGlobalPathPlanParameters.expiryDurationMicroSec = 15 * 1000000;
    mConfigFileReader.getParam(mNhPrivate, "pointRemovalRadius", mGlobalPathPlanParameters.pointRemovalRadius, 0.45);
    mConfigFileReader.getParam(mNhPrivate, "agentRadius", mGlobalPathPlanParameters.agentRadius, 0.3);
    mConfigFileReader.getParam(mNhPrivate, "waypointReachedBoundary", mGlobalPathPlanParameters.waypointReachedBoundary, 1.5);
    mConfigFileReader.getParam(mNhPrivate, "desiredHeight", mGlobalPathPlanParameters.desiredHeight, 2.0);

    // Global path plan parameters

    // PathPlanningParams()
    // : radius(0.5)
    // , cylinderHeight(1.0)
    // , mapResolution(1.0)
    // , mapMinBoundsX(-150.0)
    // , mapMaxBoundsX(150.0)
    // , mapMinBoundsY(-150.0)
    // , mapMaxBoundsY(150.0)
    // , mapMinBoundsZ(1.0)
    // , mapMaxBoundsZ(2.0)
    // , hCostWeight(1.0)
    // , potentialRadius(3.0)
    // , searchRadius(1.5)
    // , performJPS(true)
    // {}

    // Path planning parameters
    mConfigFileReader.getParam(mNhPrivate, "radius", mPathPlanningParameters.radius, 0.5);
    mConfigFileReader.getParam(mNhPrivate, "cylinderHeight", mPathPlanningParameters.cylinderHeight, 1.0);
    mConfigFileReader.getParam(mNhPrivate, "mapResolution", mPathPlanningParameters.mapResolution, 1.0);
    mConfigFileReader.getParam(mNhPrivate, "mapMinBoundsX", mPathPlanningParameters.mapMinBoundsX, -150.0);
    mConfigFileReader.getParam(mNhPrivate, "mapMaxBoundsX", mPathPlanningParameters.mapMaxBoundsX, 150);
    mConfigFileReader.getParam(mNhPrivate, "mapMinBoundsY", mPathPlanningParameters.mapMinBoundsY, -150.0);
    mConfigFileReader.getParam(mNhPrivate, "mapMaxBoundsY", mPathPlanningParameters.mapMaxBoundsY, 150.0);
    mConfigFileReader.getParam(mNhPrivate, "mapMinBoundsZ", mPathPlanningParameters.mapMinBoundsZ, 1.0);
    mConfigFileReader.getParam(mNhPrivate, "mapMaxBoundsZ", mPathPlanningParameters.mapMaxBoundsZ, 2.0);
    mConfigFileReader.getParam(mNhPrivate, "hCostWeight", mPathPlanningParameters.hCostWeight, 1.0);
    mConfigFileReader.getParam(mNhPrivate, "potentialRadius", mPathPlanningParameters.potentialRadius, 3.0);
    mConfigFileReader.getParam(mNhPrivate, "searchRadius", mPathPlanningParameters.searchRadius, 1.5);
    mConfigFileReader.getParam(mNhPrivate, "performJPS", mPathPlanningParameters.performJPS, true);
}


void TeamingPlanner::printOutDroneMapVariables()
{
    ROS_INFO("[Teaming Planner %d]: mAgentsPoseMap_cp size : %d!", mSourceSegmentId, mAgentsPoseMap_cp.size());
    ROS_INFO("[Teaming Planner %d]: mAgentsPhasesAndTimeMap_cp size : %d!", mSourceSegmentId, mAgentsPhasesAndTimeMap_cp.size());
    // ROS_INFO("[Teaming Planner %d]: mAgentsPathAndWaypointProgressMap_cp size : %d!", mSourceSegmentId, mAgentsPathAndWaypointProgressMap_cp.size());
    // ROS_INFO("[Teaming Planner %d]: mAgentsPlannedPathMap_cp size : %d!", mSourceSegmentId, mAgentsPlannedPathMap_cp.size());
    // ROS_INFO("[Teaming Planner %d]: mAgentsProcessedPathOfAgentsMap_cp size : %d!", mSourceSegmentId, mAgentsProcessedPathOfAgentsMap_cp.size());

    ROS_INFO("[Teaming Planner %d]: mAgentsPhaseAndTimeMap_rf size : %d!", mSourceSegmentId, mAgentsPhaseAndTimeMap_rf.size());
    ROS_INFO("[Teaming Planner %d]: mAgentsPoseMap_rf size : %d!", mSourceSegmentId, mAgentsPoseMap_rf.size());
    // ROS_INFO("[Teaming Planner %d]: mAgentsDirectionUtilityMap_rf size : %d!", mSourceSegmentId, mAgentsDirectionUtilityMap_rf.size());
    // ROS_INFO("[Teaming Planner %d]: mAgentsConvexRegion2DMap_rf size : %d!", mSourceSegmentId, mAgentsConvexRegion2DMap_rf.size());
    // ROS_INFO("[Teaming Planner %d]: mAgentsConvexRegion3DMap_rf size : %d!", mSourceSegmentId, mAgentsConvexRegion3DMap_rf.size());
    // ROS_INFO("[Teaming Planner %d]: mAgentsAssignedVirtualPoseMap_rf size : %d!", mSourceSegmentId, mAgentsAssignedVirtualPoseMap_rf.size());
}