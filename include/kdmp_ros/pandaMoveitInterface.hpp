#ifndef PANDA_MOVEIT_INTERFACE_
#define PANDA_MOVEIT_INTERFACE_

#include <ros/ros.h>

#include <moveit/robot_state/conversions.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/GetPositionIK.h>
#include <moveit_msgs/GetPositionFK.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/collision_detection_bullet/collision_env_bullet.h>
#include <moveit/collision_detection_bullet/collision_detector_allocator_bullet.h>
#include <moveit/collision_detection/collision_tools.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit/robot_state/conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <moveit/utils/robot_model_test_utils.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>
#include <RobotInterface.hpp>
#include "kdmp_ros/PandaControlCmd.h"


#include <queue>
#include <mutex>

class PandaMoveitInterface : public RobotInterface
{
    public:
        PandaMoveitInterface(ros::NodeHandle nh);
        virtual std::vector<double> inverseKinematics(std::vector<double> eePose);
        virtual bool inCollision(std::vector<double> q);
        virtual void setRobotState(std::vector<double> state);
        virtual void sendControlCommand(std::vector<double> controlCommand);
        virtual std::vector<double> getRandomConfig(void);
        virtual std::vector<double> forwardKinematics(std::vector<double> q);
        virtual std::vector<double> eeVelToJointVel(std::vector<double> eeVel, std::vector<double> q);
        virtual std::vector<double> jointVelToEeVel(std::vector<double> qd, std::vector<double> q);
        virtual Eigen::MatrixXd getJacobian(std::vector<double> q);
        virtual std::vector<double> ddqFromEEAcc(std::vector<double> ee_acc, std::vector<double> dq, std::vector<double> q);
        virtual std::vector<double> sampleControl();
        std::vector<double> sampleControlTest(kdmp_ros::PandaControlCmd &cmd);

    private:
        ros::NodeHandle nh_;
        ros::ServiceClient ik_service_client_;
        ros::ServiceClient fk_service_client_;
        ros::Subscriber control_sample_sub_;
        void controlCallback(const kdmp_ros::PandaControlCmd::ConstPtr &msg);

        ros::Publisher robot_state_publisher_;
        planning_scene::PlanningScenePtr planning_scene_;
        robot_model::RobotModelPtr robot_model_;
        moveit_msgs::DisplayRobotState currentState_;
        robot_model_loader::RobotModelLoader robot_model_loader_;
        moveit::core::RobotModelPtr kinematic_model_;
        moveit::core::RobotStatePtr kinematic_state_;
        const moveit::core::JointModelGroup* joint_model_group_;
        const moveit::core::JointModelGroup* joint_model_group_hand_;

        random_numbers::RandomNumberGenerator rng_;
        std::queue<kdmp_ros::PandaControlCmd> control_queue_;
        unsigned int control_queue_limit_;
        std::mutex control_queue_mutex_;

};

#endif