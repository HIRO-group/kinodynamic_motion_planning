#include <ros/ros.h>

#include <kdmp_ros/pandaMoveitInterface.hpp>
#include <kdmp/spaces/include/pandaConstants.hpp>
#include "kdmp/problems/include/pandaSetup.hpp"
#include <kdmp/problems/include/pandaGoal.hpp>

#include <string>
#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ompl_rrt");

    ros::NodeHandle n;
    std::shared_ptr<RobotInterface> robot_interface = std::make_shared<PandaMoveitInterface>(n);
    std::vector<double> startVec(0.0, 2 * PANDA_NUM_JOINTS);
    std::string plannerType = "rrt";
    PandaSetup setup(plannerType, robot_interface, startVec);
    return 0;
}