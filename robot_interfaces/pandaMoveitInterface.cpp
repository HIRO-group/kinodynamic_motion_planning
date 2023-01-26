#include "kdmp_ros/pandaMoveitInterface.hpp"

PandaMoveitInterface::PandaMoveitInterface(ros::NodeHandle nh) : nh_(nh)
{

}

bool PandaMoveitInterface::inCollision(std::vector<double> q)
{
    return false;
}

std::vector<double> PandaMoveitInterface::inverseKinematics(std::vector<double> eePose)
{
    return std::vector<double>();
}

void PandaMoveitInterface::setRobotState(std::vector<double> state)
{
    //no op
}

void PandaMoveitInterface::sendControlCommand(std::vector<double> controlCommand)
{
    // no op
}