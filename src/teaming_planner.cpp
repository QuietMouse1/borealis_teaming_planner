#include "../include/teaming_planner/teaming_planner.h"

TeamingPlanner::TeamingPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nhPrivate):
        mNh(nh),
        mNhPrivate(nhPrivate),
        mConfigFileReader(),
        mModuleState(TeamingPlannerConstants::ModuleState::INITILAISING),
        mAgentsPhaseAndTimeMap_rf(),
        mAgentsPoseMap_rf(),
        mAgentsDirectionUtilityMap_rf(),
        mAgentsConvexRegion2DMap_rf(),
        mAgentsAssignedVirtualPoseMap_rf(),
        // mPointCloudTransformListener(),
        mTask(),
        mHistoryOfHumanPoses_rf(),
        mHistoryOfHumanPosesReceived(false),
        mModuleStateVerbose(false),
        mModuleTaskVerbose(false),
        mPointcloudCallbackVerbose(false),
        mAgentPoseCallbackVerbose(false),
        mSystemPoseCallbackVerbose(false),
        mPhaseSyncCallbackVerbose(false),
        mDebugVerbose(true),
        mTeamSize(0),
        mHandlerPtr(std::make_shared<DistributedFormation::DistributedMultiRobotFormationHandler>()),
        mGlobalPathPlannerHandlerPtr(std::make_shared<DistributedGlobalPathPlanner::DistributedGlobalPathPlannerHandler>()),
        mAgentsPoseMap_cp(),
        mAgentsPhasesAndTimeMap_cp(),
        mAgentsPathAndWaypointProgressMap_cp(),
        mAgentsPlannedPathMap_cp(),
        mAgentsProcessedPathOfAgentsMap_cp(),
        // mOwnProcessedPathOfAgents_cp(),
        mAgentsBestProcessedPath_cp()
    {
        // Module Configurable Variables

        mtfListener = new tf2_ros::TransformListener(mtfBuffer); // guess and checked until code compiled xd

        mConfigFileReader.getParam(nhPrivate, "sourceSegmentId", mSourceSegmentId, static_cast<uint32_t>(0));
        mConfigFileReader.getParam(nhPrivate, "modulePeriod", mModulePeriod, 1);
        mConfigFileReader.getParam(nhPrivate, "numOfAgents", mNumOfAgents, 3);
        mConfigFileReader.getParam(nhPrivate, "debugVerbose", mDebugVerbose, false);
        mConfigFileReader.getParam(nhPrivate, "intervalDistance", mIntervalDistance, 0.5);
        mConfigFileReader.getParam(nhPrivate, "planningHorizon", mPlanningHorizon, 25);
        mConfigFileReader.getParam(nhPrivate, "desiredHeight", mDesiredHeight, 1.2);
        mConfigFileReader.getParam(nhPrivate, "useUWB", mUseUWB, false);
        mConfigFileReader.getParam(nhPrivate, "UseOccupancyMap", mUseOccupancyMap, false);
        mConfigFileReader.getParam(nhPrivate, "pointcloudCallbackVerbose", mPointcloudCallbackVerbose, false);
        mConfigFileReader.getParam(nhPrivate, "agentPoseCallbackVerbose", mAgentPoseCallbackVerbose, false);
        mConfigFileReader.getParam(nhPrivate, "systemPoseCallbackVerbose", mSystemPoseCallbackVerbose, false);
        mConfigFileReader.getParam(nhPrivate, "phaseSyncCallbackVerbose", mPhaseSyncCallbackVerbose, false);

        // Start parameter server
        dynamic_reconfigure::Server<borealis_teaming_planner::borealis_teaming_planner_Config> server;
        dynamic_reconfigure::Server<borealis_teaming_planner::borealis_teaming_planner_Config>::CallbackType f;
        
        f = boost::bind(&TeamingPlanner::paramServerCallback, this ,_1, _2);
        server.setCallback(f);

        // Subscribers
        if (mUseUWB)
        {
            mSelfSystemPoseSubscriber = mNh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/system_pose_uwb", 10, &TeamingPlanner::selfSystemPoseCallbackUWB, this);
        }
        else
        {
            mSelfSystemPoseSubscriber = mNh.subscribe<geometry_msgs::PoseStamped>("/system_pose", 10, &TeamingPlanner::selfSystemPoseCallback, this);
        }

        if (mUseOccupancyMap) // mUseOccupancyMap pcl1 data type no need 2 transform
        {
            mSystemPointCloudSubscriber = mNh.subscribe<sensor_msgs::PointCloud>("/pointcloudmap", 10, &TeamingPlanner::systemPointCloudCallback, this);
        }
        else
        {
            mSystemPointCloud2Subscriber = mNh.subscribe<sensor_msgs::PointCloud2>("/pointcloud", 10, &TeamingPlanner::systemPointCloud2Callback, this);
        }

        mActivatePlannerSubscriber = mNh.subscribe<std_msgs::Bool>("/activate_planner", 10, &TeamingPlanner::activatePlannerCallback, this);

        // Publishers
        // mVoxelFilterCloudPublisher_rf = mNh.advertise<sensor_msgs::PointCloud>("/voxel_filter_cloud",10);

        /* Robot formation handler publishers */
        mPhaseAndTimePublisher_rf = mNh.advertise<mt_msgs::phaseAndTime>("/phase_and_time_rf", 10);
        mPosePublisher_rf = mNh.advertise<mt_msgs::pose>("/system_pose_rf", 10);
        mDirectionUtilityPublisher_rf = mNh.advertise<mt_msgs::angleIndexAndUtility>("/direction_utility_rf", 10);
        mConvexRegion2DPublisher_rf = mNh.advertise<mt_msgs::convexRegion2D>("/convex_region_2D_rf", 10);
        mConvexRegion3DPublisher_rf = mNh.advertise<mt_msgs::convexRegion3D>("/convex_region_3D_rf", 10);
        mAssignedVirtualPosePublisher_rf = mNh.advertise<geometry_msgs::PoseStamped>("/assigned_virtual_position_rf", 10);
        mAssignedVirtualPoseMapPublisher_rf = mNh.advertise<mt_msgs::posevector>("/assigned_virtual_pose_map_rf", 10); // ctrl f, virtual pose map is not published

        /* Consensus path handler publisher */
        mPhaseAndTimePublisher_cp = mNh.advertise<mt_msgs::phaseAndTime>("/phase_and_time_cp", 10);
        mPosePublisher_cp = mNh.advertise<mt_msgs::pose>("/system_pose_cp", 10);
        mPathAndOwnWayPointProgressPublisher_cp = mNh.advertise<mt_msgs::pathAndProgress>("/path_and_progress_cp", 10);
        mOwnPlannedPathPublisher_cp = mNh.advertise<mt_msgs::posevector>("/plannned_path_cp", 10);
        mOwnProcessedPathOfAgentsPublisher_cp = mNh.advertise<mt_msgs::pathAndCostVector>("/processed_path_of_agents_cp", 10); 
        mOwnBestProcessedPath_cpPublisher_cp = mNh.advertise<mt_msgs::posevector>("/best_processed_path_cp", 10);
        mProcessedGoTherePathPublisher_cp = mNh.advertise<geometry_msgs::PoseArray>("/processed_go_there_path", 10); 

        // Subscribers 
        mGoalSubscriber = mNh.subscribe<mt_msgs::pose>("/goal", 10, &TeamingPlanner::goalCallback, this);
        mProcessedGoTherePathSubscriber = mNh.subscribe<geometry_msgs::PoseArray>("/processed_go_there_path", 10, &TeamingPlanner::ProcessedGoTherePathCallback, this);
        // mHumanSystemPoseSubscriber = mNh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/human_input_pose", 10, &TeamingPlanner::humanSystemPoseCallback, this);
        mTeamingAgentRadiusSubscriber = mNh.subscribe<std_msgs::Float32>("/teaming_agent_radius", 10, &TeamingPlanner::teamingAgentRadiusCallback, this);

        for (int i = 1; i <= mNumOfAgents; i++)
        {
            if (i == mSourceSegmentId)
            {
                std::string UAVModeTopic = "/uav" + std::to_string(i) + "/hri_mode";
                std::string inputUAVPoseStampedTopic = "/uav" + std::to_string(i) + "/input_pose_stamped";
                std::string numberOfAgentsTopic = "/uav" + std::to_string(i) + "/number_of_agents_in_team";

                mUAVmodeSubscriber = mNh.subscribe<std_msgs::String>(UAVModeTopic, 10, &TeamingPlanner::UAVModeCallback, this);
                mInputUAVPoseStampedSubscriber = mNh.subscribe<geometry_msgs::PoseStamped>(inputUAVPoseStampedTopic, 10, &TeamingPlanner::UAVInputPoseStampedCallback, this);
                mAgentsInTeamVectorSubscriber = mNh.subscribe<std_msgs::Int8MultiArray> (numberOfAgentsTopic, 10, &TeamingPlanner::numberOfAgentsInTeamCallback, this);
                continue;
            }

            /* Robot formation handler subsribers */
            std::string phaseAndTimeTopic_rf = "/uav" + std::to_string(i) + "/borealis_teaming_planner/phase_and_time_rf";
            std::string systemPoseTopic_rf = "/uav" + std::to_string(i) + "/borealis_teaming_planner/system_pose_rf";
            std::string directionUtilityTopic_rf = "/uav" + std::to_string(i) + "/borealis_teaming_planner/direction_utility_rf";
            std::string convexRegion2DTopic_rf = "/uav" + std::to_string(i) + "/borealis_teaming_planner/convex_region_2D_rf";
            std::string convexRegion3DTopic_rf = "/uav" + std::to_string(i) + "/borealis_teaming_planner/convex_region_3D_rf";
            std::string assignedVirtualPoseMapTopic_rf = "/uav" + std::to_string(i) + "/borealis_teaming_planner/assigned_virtual_pose_map_rf";

            ros::Subscriber phaseAndTimeSubscriber_rf = mNh.subscribe<mt_msgs::phaseAndTime>(phaseAndTimeTopic_rf, 10, &TeamingPlanner::phaseTimeCallback_rf, this);
            ros::Subscriber systemPoseSubscriber_rf = mNh.subscribe<mt_msgs::pose>(systemPoseTopic_rf, 10, &TeamingPlanner::systemPoseCallback_rf, this);
            ros::Subscriber directionUtilitySubscriber_rf = mNh.subscribe<mt_msgs::angleIndexAndUtility>(directionUtilityTopic_rf, 10, &TeamingPlanner::directionUtilityCallback_rf, this);
            ros::Subscriber convexRegion2DSubscriber_rf = mNh.subscribe<mt_msgs::convexRegion2D>(convexRegion2DTopic_rf, 10, &TeamingPlanner::convexRegion2DCallback_rf, this);
            ros::Subscriber convexRegion3DSubscriber_rf = mNh.subscribe<mt_msgs::convexRegion3D>(convexRegion3DTopic_rf, 10, &TeamingPlanner::convexRegion3DCallback_rf, this);
            ros::Subscriber convexRegion3DSuassignedVirtualPoseMapSubscriber_rf = mNh.subscribe<mt_msgs::posevector>(assignedVirtualPoseMapTopic_rf, 10, &TeamingPlanner::assignedVirtualPoseMapCallback_rf, this);

            mUAVPhaseAndTimeSubscriberVector_rf.push_back(phaseAndTimeSubscriber_rf);
            mUAVSystemPoseSubscriberVector_rf.push_back(systemPoseSubscriber_rf);
            mUAVDirectionUtilitySubscriberVector_rf.push_back(directionUtilitySubscriber_rf);
            mUAVConvexRegion2DSubscriberVector_rf.push_back(convexRegion2DSubscriber_rf);
            mUAVConvexRegion3DSubscriberVector_rf.push_back(convexRegion3DSubscriber_rf);
            mUAVAssignedVirtualPoseMapSubscriberVector_rf.push_back(convexRegion3DSuassignedVirtualPoseMapSubscriber_rf);

            /* Consensus global path subsribers */
            std::string phaseTimeTopic_cp = "/uav" + std::to_string(i) + "/borealis_teaming_planner/phase_and_time_cp";
            std::string systemPoseTopic_cp = "/uav" + std::to_string(i) + "/borealis_teaming_planner/system_pose_cp";
            std::string pathAndProgressTopic_cp = "/uav" + std::to_string(i) + "/borealis_teaming_planner/path_and_progress_cp";
            std::string plannedPathTopic_cp = "/uav" + std::to_string(i) + "/borealis_teaming_planner/plannned_path_cp";
            std::string processedPathTopic_cp = "/uav" + std::to_string(i) + "/borealis_teaming_planner/processed_path_cp";
            std::string bestProcessedTopic_cp = "/uav" + std::to_string(i) + "/borealis_teaming_planner/best_processed_path_cp";
            std::string agentsBestProcessedPathTopic_cp = "/uav" + std::to_string(i) + "/borealis_teaming_planner/processed_path_of_agents_cp";

            ros::Subscriber phaseAndTimeSubscriber_cp = mNh.subscribe<mt_msgs::phaseAndTime>(phaseTimeTopic_cp, 10, &TeamingPlanner::phaseTimeCallback_cp, this);
            ros::Subscriber UAVSystemPoseSubscriber_cp = mNh.subscribe<mt_msgs::pose>(systemPoseTopic_cp, 10, &TeamingPlanner::systemPoseCallback_cp, this);
            ros::Subscriber pathAndWayPointProgressSubscriber_cp = mNh.subscribe<mt_msgs::pathAndProgress>(pathAndProgressTopic_cp, 10, &TeamingPlanner::pathAndProgressCallback_cp, this);
            ros::Subscriber plannedPathSubscriber_cp = mNh.subscribe<mt_msgs::posevector>(plannedPathTopic_cp, 10, &TeamingPlanner::plannedPathCallback_cp, this);           
            ros::Subscriber agentsBestProcessedPathSubscriber_cp = mNh.subscribe<mt_msgs::pathAndCostVector>(agentsBestProcessedPathTopic_cp, 10, &TeamingPlanner::agentProcessedPathOfAgentsCallback_cp, this);
            ros::Subscriber bestProcessedPathSubsriber_cp = mNh.subscribe<mt_msgs::posevector>(bestProcessedTopic_cp, 10, &TeamingPlanner::agentBestProcessedPathCallback_cp, this);

            mUAVSystemPoseSubscriberVector_cp.push_back(UAVSystemPoseSubscriber_cp);
            mUAVPhaseAndTimeSubscriberVector_cp.push_back(phaseAndTimeSubscriber_cp);
            pathAndWayPointProgressSubscriberVector_cp.push_back(pathAndWayPointProgressSubscriber_cp);
            plannedPathSubscriberVector_cp.push_back(plannedPathSubscriber_cp);
            agentsBestProcessedPathSubscriberVector_cp.push_back(agentsBestProcessedPathSubscriber_cp);
            bestProcessedPathSubsriberVector_cp.push_back(bestProcessedPathSubsriber_cp);
            // processedPathSubsriberVector_cp.push_back(processedPathSubsriber_cp);
        }
        
        // Timers
        // mModuleLoopTimer = mNh.createTimer(ros::Duration(mModulePeriod), &TeamingPlanner::moduleLoopCallback, this);

        ros::Duration(0.5).sleep(); // work around to allow tf buffers to fill up and preven throwing of tf buffer length exception

        ros::Rate loop_rate(25);
        while (ros::ok())
        {
            teamingPlannerMain();
            ros::spinOnce();
            loop_rate.sleep();
        }
    }

TeamingPlanner::~TeamingPlanner()
{
    // Destructor
}
