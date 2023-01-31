#include <ros/ros.h>

// #include <kdmp/problems/include/RobotInterface.hpp>
// #include <kdmp_ros/pandaMoveitInterface.hpp>
// #include <kdmp/spaces/include/pandaConstants.hpp>
// #include "kdmp/problems/include/pandaSetup.hpp"
// #include <kdmp/problems/include/pandaGoal.hpp>

#include "pandaSetup.hpp"
#include "pandaConstants.hpp"
#include "pandaMoveitInterface.hpp"
#include "pandaGoal.hpp"

#include <string>
#include <vector>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ompl_rrt");

    ros::NodeHandle n;
    std::shared_ptr<RobotInterface> robot_interface = std::make_shared<PandaMoveitInterface>(n);
    std::vector<double> startVec(2 * PANDA_NUM_JOINTS, 0.0);
    std::string plannerType = "rrt";
    PandaSetup setup(plannerType.c_str(), robot_interface, startVec);
    return 0;
}