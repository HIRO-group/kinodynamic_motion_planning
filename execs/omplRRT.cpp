#include <ros/ros.h>

#include "kdmp/problems/include/pandaMPWithOptimization.hpp"
#include <kdmp_ros/pandaMoveitInterface.hpp>

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ompl_rrt");

    ros::NodeHandle n;
    PandaMoveitInterface robot_interface(n);
    return 0;
}