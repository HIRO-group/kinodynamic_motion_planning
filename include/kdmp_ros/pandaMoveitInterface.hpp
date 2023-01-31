#ifndef PANDA_MOVEIT_INTERFACE_
#define PANDA_MOVEIT_INTERFACE_

#include <ros/ros.h>
#include <RobotInterface.hpp>
#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit/robot_state/conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/utils/robot_model_test_utils.h>

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
        planning_scene::PlanningScenePtr planning_scene_;
        robot_model::RobotModelPtr robot_model_;
        moveit_msgs::DisplayRobotState currentState_;

};

#endif