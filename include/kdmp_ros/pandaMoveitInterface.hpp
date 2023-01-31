#ifndef PANDA_MOVEIT_INTERFACE_
#define PANDA_MOVEIT_INTERFACE_

#include <ros/ros.h>
#include <RobotInterface.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/GetPositionIK.h>

class PandaMoveitInterface : public RobotInterface
{
    public:
        PandaMoveitInterface(ros::NodeHandle nh);
        virtual std::vector<double> inverseKinematics(std::vector<double> eePose);
        virtual bool inCollision(std::vector<double> q);
        virtual void setRobotState(std::vector<double> state);
        virtual void sendControlCommand(std::vector<double> controlCommand);

    private:
        ros::NodeHandle nh_;
        ros::ServiceClient ik_service_client_;
        ros::Publisher robot_state_publisher_;

};

#endif