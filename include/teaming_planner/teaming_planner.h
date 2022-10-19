#ifndef TEAMING_PLANNER_H
#define TEAMING_PLANNER_H

#include <ros/ros.h>
#include <ros/subscribe_options.h>
#include <unordered_map>
#include <string>
#include <algorithm>

#include "../../../distributed_multi_robot_formation/src/Common/Common.h"
#include "../../../distributed_multi_robot_formation/src/DistributedMultiRobotFormation.h"
#include "../../../distributed_multi_robot_formation/src/DistributedMultiRobotFormationHandler.h"
#include "../../../distributed_multi_robot_formation/src/ProcessPointCloud/ProcessPointCloud.h"

#include "../../../distributed_global_path_planner/src/DistributedGlobalPathPlannerHandler.h"
#include "../../../distributed_global_path_planner/src/DistributedGlobalPathPlanner.h"

#include "../../../Common/ConstantsEnum.h"
#include "../../../Common/Config/ConfigFileReader.h"
#include "teaming_planner_constants.h"

#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>

#include <nav_msgs/Odometry.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <std_msgs/Int8.h>
#include <std_msgs/Int8MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Float32.h>

// #include <tf/transform_listener.h>
#include <mt_msgs/pose.h>
#include <mt_msgs/mtTask.h>
#include <mt_msgs/angleIndexAndUtility.h>
#include <mt_msgs/convexRegion2D.h>
#include <mt_msgs/convexRegion3D.h>
#include <mt_msgs/phaseAndTime.h>
#include <mt_msgs/position.h>
#include <mt_msgs/posevector.h>
#include <mt_msgs/pathAndProgress.h>
#include <mt_msgs/pathAndCost.h>
#include <mt_msgs/pathAndCostVector.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf2_ros/transform_listener.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2/convert.h>
#include <tf2_ros/buffer.h>
#include <tf2/transform_datatypes.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <dynamic_reconfigure/server.h>
#include <borealis_teaming_planner/borealis_teaming_planner_Config.h>
 

// Can remove goalPose callback?

// Teamping planner is still programmed to take in only 1 got there input.
// If multiple go there input is passed, only the latest one will be considered.

// void callback(parameter_server_basics::parameter_server_Config &config, uint32_t level) {
//   ROS_INFO("Reconfigure Request: %d %f %s %s %d", 
//             config.int_param, config.double_param, 
//             config.str_param.c_str(), 
//             config.bool_param?"True":"False", 
//             config.size);
// }
 
// int main(int argc, char **argv) {
//   ros::init(argc, argv, "parameter_server_basics");
 
//   dynamic_reconfigure::Server<parameter_server_basics::parameter_server_Config> server;
//   dynamic_reconfigure::Server<parameter_server_basics::parameter_server_Config>::CallbackType f;
 
//   f = boost::bind(&callback, _1, _2);
//   server.setCallback(f);
 
//   ROS_INFO("Spinning node");
//   ros::spin();
//   return 0;
// }


class TeamingPlanner
{
    private:

        ros::NodeHandle mNh;
        ros::NodeHandle mNhPrivate;
        Common::Utils::ConfigFileReader mConfigFileReader;

        DistributedFormation::DistributedMultiRobotFormationHandler::Ptr mHandlerPtr;
        DistributedFormation::DistributedMultiRobotFormation mDistributedFormation;
        
        DistributedGlobalPathPlanner::DistributedGlobalPathPlannerHandler::Ptr mGlobalPathPlannerHandlerPtr;
        DistributedGlobalPathPlanner::DistributedGlobalPathPlanner mGlobalPathPlanner;
        
        // std::unique_ptr<DistributedGlobalPathPlanner::DistributedGlobalPathPlanner> mGlobalPathPlannerPtr(new DistributedGlobalPathPlanner::DistributedGlobalPathPlanner());
        // DistributedGlobalPathPlanner::DistributedGlobalPathPlanner* mGlobalPathPlannerPtr;


        // Configurable Variables
        uint32_t mSourceSegmentId;
        double mModulePeriod;
        int mNumOfAgents;
        bool mDebugVerbose;
        bool mPointcloudCallbackVerbose;
        bool mAgentPoseCallbackVerbose;
        bool mSystemPoseCallbackVerbose;
        bool mPhaseSyncCallbackVerbose;

        double mIntervalDistance;
        double mPlanningHorizon;
        double mDesiredHeight;
        DistributedFormation::Common::DistributedFormationParameters mRobotFormationParameters;
        DistributedGlobalPathPlanner::Common::DistributedGlobalPathParams mGlobalPathPlanParameters;
        pathplanning::PathPlanningParams mPathPlanningParameters;
    
        // Teaming Planner control variables
        TeamingPlannerConstants::ModuleState mModuleState;
        Common::Entity::MTTaskBundle mTask;

        // Drone variables
        sensor_msgs::PointCloud mSystemPointCloud;
        geometry_msgs::PoseStamped mSelfSystemPose;
        sensor_msgs::PointCloud2 mSystemPointCloud2;
        // tf::TransformListener mPointCloudTransformListener;
        // tf::TransformListener mPointCloud2TransformListener;
        bool mHistoryOfHumanPosesReceived;
        bool mUseUWB;
        bool mUseOccupancyMap;

        std_msgs::Bool mBoolActivatePlanner; 
        geometry_msgs::PoseStamped mInputUAVPoseStamped;

        tf2_ros::Buffer mtfBuffer;
        tf2_ros::TransformListener* mtfListener; // Some C++ stuff check it out https://answers.ros.org/question/80206/why-cant-initialize-tf2_rostransformlistenter-in-hydro/

        std::string mUAVMode;
        std::vector<int> mAgentsInTeamVector;
        std_msgs::Int8MultiArray mAgentsInTeam;
        int mTeamSize;

        // Verbose Variables
        bool mModuleStateVerbose;
        bool mModuleTaskVerbose;

        // Subscribers 
        ros::Subscriber mGoalSubscriber;
        ros::Subscriber mHumanSystemPoseSubscriber;
        ros::Subscriber mSelfSystemPoseSubscriber;
        ros::Subscriber mSystemPointCloudSubscriber;
        ros::Subscriber mSystemPointCloud2Subscriber;
        ros::Subscriber mActivatePlannerSubscriber;
        ros::Subscriber mUAVmodeSubscriber;
        ros::Subscriber mInputUAVPoseStampedSubscriber;
        ros::Subscriber mAgentsInTeamVectorSubscriber;
        ros::Subscriber mProcessedGoTherePathSubscriber;
        ros::Subscriber mTeamingAgentRadiusSubscriber;
                    
        /* Multi Robot formation variables used by handler funcs*/
        std::vector<DistributedFormation::Common::Pose> mHistoryOfHumanPoses_rf;
        std::vector<DistributedFormation::Common::Pose> mProcessedGoTherePath;
        DistributedFormation::Common::Pose mHumanSystemPose_rf;
        DistributedFormation::Common::Pose mSelfSystemPose_rf;
        std::unordered_map<int32_t, DistributedFormation::Common::PhaseAndTime> mAgentsPhaseAndTimeMap_rf;
        std::unordered_map<int32_t, DistributedFormation::Common::Pose> mAgentsPoseMap_rf;
        std::unordered_map<int32_t, DistributedFormation::Common::DirectionUtility> mAgentsDirectionUtilityMap_rf;
        std::unordered_map<int32_t, DistributedFormation::Common::ConvexRegion2D> mAgentsConvexRegion2DMap_rf;
        std::unordered_map<int32_t, DistributedFormation::Common::ConvexRegion3D> mAgentsConvexRegion3DMap_rf;
        std::unordered_map<int32_t, std::unordered_map<int32_t, DistributedFormation::Common::Pose>> mAgentsAssignedVirtualPoseMap_rf;
    
        /* Publishers used by RFH*/
        ros::Publisher mPhaseAndTimePublisher_rf;
        ros::Publisher mPosePublisher_rf;
        ros::Publisher mDirectionUtilityPublisher_rf;
        ros::Publisher mConvexRegion2DPublisher_rf;
        ros::Publisher mConvexRegion3DPublisher_rf;
        ros::Publisher mAssignedVirtualPosePublisher_rf;
        ros::Publisher mAssignedVirtualPoseMapPublisher_rf;
        // ros::Publisher mVoxelFilterCloudPublisher_rf;

        /* Subcribers used by RFH*/
        // Hardcoded for now
        std::vector<ros::Subscriber> mUAVSystemPoseSubscriberVector_rf;
        std::vector<ros::Subscriber> mUAVPhaseAndTimeSubscriberVector_rf;
        std::vector<ros::Subscriber> mUAVDirectionUtilitySubscriberVector_rf;
        std::vector<ros::Subscriber> mUAVConvexRegion2DSubscriberVector_rf;
        std::vector<ros::Subscriber> mUAVConvexRegion3DSubscriberVector_rf;
        std::vector<ros::Subscriber> mUAVAssignedVirtualPoseMapSubscriberVector_rf;

        /* Consensus global path variables */
        DistributedGlobalPathPlanner::Common::Pose mOwnAgentPose_cp;
        std::vector<DistributedGlobalPathPlanner::Common::Pose> mGoTherePath_cp;
        DistributedGlobalPathPlanner::Common::PhaseAndTime mOwnAgentPhaseAndTime_cp;
        DistributedGlobalPathPlanner::Common::PathAndWaypointProgress mOwnPathAndWaypointProgress_cp;
        std::vector<Eigen::Vector3d> mOwnPlannedPath_cp;
        std::vector<Eigen::Vector3d> mOwnBestProcessedPath_cp;
        std::vector<DistributedGlobalPathPlanner::Common::Pose> m_ownProcessedGoTherePath_cp;
        std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::Pose> mAgentsPoseMap_cp;
        std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PhaseAndTime> mAgentsPhasesAndTimeMap_cp;
        std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PathAndWaypointProgress> mAgentsPathAndWaypointProgressMap_cp;
        std::unordered_map<int32_t, std::vector<Eigen::Vector3d>> mAgentsPlannedPathMap_cp;
        std::unordered_map<int32_t, std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PathAndCost>> mAgentsProcessedPathOfAgentsMap_cp;
        // std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PathAndCost> mOwnProcessedPathOfAgents_cp;
        std::unordered_map<int32_t, std::vector<Eigen::Vector3d>> mAgentsBestProcessedPath_cp;

        /* Publishers used by Consensus global path handler*/
        ros::Publisher mPosePublisher_cp;
        ros::Publisher mPhaseAndTimePublisher_cp;
        ros::Publisher mOwnPlannedPathPublisher_cp; 
        ros::Publisher mPathAndOwnWayPointProgressPublisher_cp; 
        ros::Publisher mOwnProcessedPathOfAgentsPublisher_cp; 
        ros::Publisher mOwnBestProcessedPath_cpPublisher_cp;
        ros::Publisher mProcessedGoTherePathPublisher_cp; // Final output

        /* Subcribers for Consesus path */
        std::vector<ros::Subscriber> mUAVSystemPoseSubscriberVector_cp;
        std::vector<ros::Subscriber> mUAVPhaseAndTimeSubscriberVector_cp;
        std::vector<ros::Subscriber> pathAndWayPointProgressSubscriberVector_cp;
        std::vector<ros::Subscriber> plannedPathSubscriberVector_cp;
        std::vector<ros::Subscriber> bestProcessedPathSubsriberVector_cp;
        std::vector<ros::Subscriber> agentsBestProcessedPathSubscriberVector_cp;
        
        // Timers
        ros::Timer mModuleLoopTimer;

        // Subscriber Callbacks
        void systemPointCloud2Callback(const sensor_msgs::PointCloud2::ConstPtr& aSystemPointCloud2);
        void systemPointCloudCallback(const sensor_msgs::PointCloud::ConstPtr& aSystemPointCloud);
        void selfSystemPoseCallback(const geometry_msgs::PoseStamped::ConstPtr& aSelfSystemPose);
        void selfSystemPoseCallbackUWB(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& aSelfSystemPose);
        void activatePlannerCallback(const std_msgs::Bool::ConstPtr& aBoolActivatePlanner);
        void UAVModeCallback(const std_msgs::String::ConstPtr& aUAVmode);
        void UAVInputPoseStampedCallback(const geometry_msgs::PoseStamped::ConstPtr& aInputPose);
        void numberOfAgentsInTeamCallback(const std_msgs::Int8MultiArray::ConstPtr& aNumberOfAgents);
        void ProcessedGoTherePathCallback(const geometry_msgs::PoseArray::ConstPtr& aInputPoseArray);

        void phaseTimeCallback_rf(const mt_msgs::phaseAndTime::ConstPtr& aPhaseAndTime);
        void directionUtilityCallback_rf(const mt_msgs::angleIndexAndUtility::ConstPtr& aDirectionUtility);
        void convexRegion2DCallback_rf(const mt_msgs::convexRegion2D::ConstPtr& aConvexRegion2D);
        void convexRegion3DCallback_rf(const mt_msgs::convexRegion3D::ConstPtr& aConvexRegion3D);
        void assignedVirtualPoseMapCallback_rf(const mt_msgs::posevector::ConstPtr& aAssignedVirtualPoseMap);
        void teamingAgentRadiusCallback(const std_msgs::Float32::ConstPtr& aAgentRadius);

        // void humanSystemPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& aHumanSystemPose);
        // void selfSystemPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& aSelfSystemPose);
        void systemPoseCallback_rf(const mt_msgs::pose::ConstPtr& aSystemPose);

        void phaseTimeCallback_cp(const mt_msgs::phaseAndTime::ConstPtr& aPhaseAndTime);
        void systemPoseCallback_cp(const mt_msgs::pose::ConstPtr& aSystemPose);
        void pathAndProgressCallback_cp(const mt_msgs::pathAndProgress::ConstPtr& aPathAndProgress);
        void plannedPathCallback_cp(const mt_msgs::posevector::ConstPtr& aPlannedPath);
        void agentProcessedPathOfAgentsCallback_cp(const mt_msgs::pathAndCostVector::ConstPtr& aPathAndCostVector);
        void agentBestProcessedPathCallback_cp(const mt_msgs::posevector::ConstPtr& aBestProcessedPath);

        /*Common Functions to be binded by formation and consensus path handler */
        bool getNumberOfAgentsInTeam(int32_t& numberOfAgentsInTeam);
        bool getOwnAgentId(int32_t& ownAgentID);
        bool getOwnAgentLidarPointCloud(sensor_msgs::PointCloud& cloud);

        /*Functions to be binded by formation handler RFH means robot formation handler*/
        // Publisher Functions
        bool pubPhaseAndTime_rf(const int32_t aAgentId, const DistributedFormation::Common::PhaseAndTime aPhaseAndTime);
        bool pubPose_rf(const int32_t aAgentId, const DistributedFormation::Common::Pose aPose);
        bool pubDirectionUtility_rf(const int32_t aAgentId, const DistributedFormation::Common::DirectionUtility aDirectionUtility);
        bool pubConvexRegion2D_rf(const int32_t aAgentId, const DistributedFormation::Common::ConvexRegion2D aConvexRegion2D);
        bool pubConvexRegion3D_rf(const int32_t aAgentId, const DistributedFormation::Common::ConvexRegion3D aConvexRegion3D);
        bool pubAssignedPose_rf(const int32_t aAgentId, const DistributedFormation::Common::Pose aAssignedVirtualPose);
        bool pubAssignedPoseMap_rf(const int32_t aAgentId, const std::unordered_map<int32_t, DistributedFormation::Common::Pose> aAssignedVirtualPoseMap);

        // Get Functions 
        bool getPosesForFormationToTrack_rf(std::vector<DistributedFormation::Common::Pose>& historyOfHumanPoses);
        bool getPhaseAndTimeMap_rf(std::unordered_map<int32_t, DistributedFormation::Common::PhaseAndTime>& phaseAndTimeMap);
        bool getPoseMap_rf(std::unordered_map<int32_t, DistributedFormation::Common::Pose>& poseMap);
        bool getDirectionUtilityMap_rf(std::unordered_map<int32_t, DistributedFormation::Common::DirectionUtility>& directionUtilityMap);
        bool getConvexRegion2DMap_rf(std::unordered_map<int32_t, DistributedFormation::Common::ConvexRegion2D>& convexRegion2DMap);
        bool getConvexRegion3DMap_rf(std::unordered_map<int32_t, DistributedFormation::Common::ConvexRegion3D>& convexRegion3DMap);
        bool getAssignedVirtualPoseMap_rf(std::unordered_map<int32_t, std::unordered_map<int32_t, DistributedFormation::Common::Pose>>& assignedVirtualPoseMap);
        bool getHumanSystemPose_rf(DistributedFormation::Common::Pose& aHumanSystemPose);
        bool getOwnUAVSystemPose_rf(DistributedFormation::Common::Pose& aUAVSystemPose);

        void clearPhaseAndTimeMap_rf(); // Not binded
        void clearPoseMap_rf(); // Binded twice ? might be wrong 
        void clearDirectionUtilityMap_rf();
        void clearConvexRegion2DMap_rf();
        void clearConvexRegion3DMap_rf();
        void clearAssignedVirtualPoseMap_rf(); // Not binded
        /*Functions to be binded by formation handler */

        /*Functions to be binded by consensus path planner CPH means consensus path handler*/
        // // Publisher functions
        bool pubOwnPhaseAndTime_cp(const int32_t ownAgentID, const DistributedGlobalPathPlanner::Common::PhaseAndTime ownAgentPhaseAndTime);
        bool pubOwnPoseFunc_cp(const int32_t ownAgentID, const DistributedGlobalPathPlanner::Common::Pose ownAgentPose);
        bool pubOwnPathAndWaypointProgress_cp(const int32_t ownAgentID, const DistributedGlobalPathPlanner::Common::PathAndWaypointProgress goTherePathAndWaypointProgress);
        bool pubOwnPlannedPath_cp(const int32_t ownAgentID, const std::vector<Eigen::Vector3d> ownPlannedPath);
        bool pubOwnProcessedPathOfAgents_cp(const int32_t ownAgentID, const std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PathAndCost> ownProcessedPathOfAgents);
        bool pubOwnBestProcessedPath_cp(const int32_t ownAgentID, const std::vector<Eigen::Vector3d> ownBestProcessedPath);
        bool pubProcessedGoTherePath_cp(const int32_t ownAgentID, const std::vector<DistributedGlobalPathPlanner::Common::Pose> processedGoTherePath); 

        // Get functions
        bool getGoTherePath_cp(std::vector<DistributedGlobalPathPlanner::Common::Pose>& goTherePath);
        bool getPhasesAndTimeRecordOfAgents_cp(std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PhaseAndTime>& phasesAndTimeRecordOfAgents);
        bool getOwnAgentPose_cp(DistributedGlobalPathPlanner::Common::Pose& ownAgentPose);
        bool getAgentsPose_cp(std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::Pose>& agentsPose);
        bool getAgentsPathAndWaypointProgress_cp(std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PathAndWaypointProgress>& agentsGoTherePathAndWaypointProgres);
        bool getAgentsPlannedPath_cp(std::unordered_map<int32_t, std::vector<Eigen::Vector3d>>& agentsPlannedPath);
        bool getAgentsProcessedPathOfAgents_cp(std::unordered_map<int32_t, std::unordered_map<int32_t, DistributedGlobalPathPlanner::Common::PathAndCost>>& agentsProcessedPath);
        bool getAgentsBestProcessedPath_cp(std::unordered_map<int32_t, std::vector<Eigen::Vector3d>>& agentsBestProcessedPath);

        // Clear
        void clearAgentsPoseBuffer_cp();
        void clearPhasesAndTime_cp();
        void clearAgentsProcessedPathOfAgentsBuffer_cp();
        void clearAgentsPlannedPathBuffer_cp();
        void clearAgentsPathAndWaypointProgressBuffer_cp();
        void clearAgentsBestProcessedPathBuffer_cp();
        void clearAgentNumberTeamVector();
        /*Functions to be binded by consensus path planner CPH means consensus path handler */

        // ParamServerCallback
        void paramServerCallback(borealis_teaming_planner::borealis_teaming_planner_Config &config, uint32_t level);

        // Functions 
        void clearOtherAgentsData();
        void teamingPlannerMain();
        void readParameters();
        bool checkAndAddHumanSystemPose(std::vector<DistributedFormation::Common::Pose>& historyOfHumanPoses, const DistributedFormation::Common::Pose aPose);
        void printOutDroneMapVariables();
        double euclideanDistance(const double x1, const double y1, const double x2, const double y2);
        void resetData();

        // Timer Functions 
        void moduleLoopCallback(const ros::TimerEvent& event);

        // KIV stuff
        void goalCallback(const mt_msgs::pose::ConstPtr& aGoal);
        // void taskCallback(const mt_msgs::mtTask::ConstPtr& aTask);

    public:
        TeamingPlanner(const ros::NodeHandle& nh, const ros::NodeHandle& nhPrivate);
        virtual ~TeamingPlanner();

}; // TeamingPlanner

#endif // TEAMING_PLANNER_H

