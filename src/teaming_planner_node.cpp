#include "teaming_planner.cpp"
#include "teaming_planner_middleware.cpp"
#include "teaming_planner_controller.cpp"
#include "teaming_planner_main.cpp"

int main (int argc, char** argv)
{
    ros::init(argc, argv, "teaming_planner");
    ros::NodeHandle nh("");
    ros::NodeHandle nhPrivate("~");

    std::unique_ptr<TeamingPlanner> teamingPlannerModule(new TeamingPlanner(nh, nhPrivate));

    //ros::MultiThreadedSpinner spinner(4);
    //spinner.spin();
    // ros::spin();

    return 0;
}