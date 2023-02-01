#include "kdmp_ros/pandaMoveitInterface.hpp"
#include "franka_model.h"
#include "pandaConstants.hpp"
#include "Eigen/Core"

PandaMoveitInterface::PandaMoveitInterface(ros::NodeHandle nh) 
    : nh_(nh), robot_model_(moveit::core::loadTestingRobotModel("panda")), robot_model_loader_("robot_description")
{
    ROS_DEBUG("initializing panda moveit interface");

    kinematic_model_ = robot_model_loader_.getModel();
    kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
    joint_model_group_ = kinematic_model_->getJointModelGroup("panda_arm");


    currentState_.state.joint_state.name = joint_model_group_->getJointModelNames();
    for (const auto &name : joint_model_group_->getJointModelNames()) {
        ROS_ERROR(name.c_str());
    }
    kinematic_state_->setToRandomPositions(joint_model_group_);
    kinematic_state_->copyJointGroupPositions(joint_model_group_,
                                           currentState_.state.joint_state.position);

    ik_service_client_ = nh_.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    robot_state_publisher_ =
      nh_.advertise<moveit_msgs::DisplayRobotState>("robot_state", 1);
    
    while (!ik_service_client_.exists())
    {
        ROS_INFO("Waiting for service");
        ros::Duration(1.0).sleep();
    }

    planning_scene_ = std::make_shared<planning_scene::PlanningScene>(robot_model_);
    planning_scene_->setActiveCollisionDetector(collision_detection::CollisionDetectorAllocatorBullet::create(),
                                             /* exclusive = */ true);

}

bool PandaMoveitInterface::inCollision(std::vector<double> q)
{
    robot_state::RobotState& state = planning_scene_->getCurrentStateNonConst();
    state.setToDefaultValues();
    state.setJointGroupPositions("panda_arm", Eigen::vecToEigenVec(q));
    state.update();
    robot_state::RobotState state_before(state);

    collision_detection::CollisionResult res;
    collision_detection::CollisionRequest req;
    req.contacts = true;
    planning_scene_->checkCollision(req, res);
    // ROS_INFO_STREAM_NAMED("pandaMoveitInterface", (res.collision ? "In collision." : "Not in collision."));

    return res.collision;
}

std::vector<double> PandaMoveitInterface::inverseKinematics(std::vector<double> eePose)
{
    moveit_msgs::GetPositionIK::Request service_request;
    moveit_msgs::GetPositionIK::Response service_response;

    Eigen::Quaternionf quat = Eigen::AngleAxisf(eePose[3], Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(eePose[4], Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(eePose[5], Eigen::Vector3f::UnitZ());
    

    service_request.ik_request.group_name = "panda_arm";
    service_request.ik_request.pose_stamped.header.frame_id = "panda_link0";
    service_request.ik_request.pose_stamped.pose.position.x = eePose[0];
    service_request.ik_request.pose_stamped.pose.position.y = eePose[1];
    service_request.ik_request.pose_stamped.pose.position.z = eePose[2];

    service_request.ik_request.pose_stamped.pose.orientation.x = quat.x();
    service_request.ik_request.pose_stamped.pose.orientation.y = quat.y();
    service_request.ik_request.pose_stamped.pose.orientation.z = quat.z();
    service_request.ik_request.pose_stamped.pose.orientation.w = quat.w();
    service_request.ik_request.robot_state.joint_state = currentState_.state.joint_state;

    service_request.ik_request.avoid_collisions = true;
    ik_service_client_.call(service_request, service_response);

    ROS_INFO_STREAM(
      "Result: " << ((service_response.error_code.val == service_response.error_code.SUCCESS) ? "True " : "False ")
                 << service_response.error_code.val);

    return service_response.solution.joint_state.position;
}

void PandaMoveitInterface::setRobotState(std::vector<double> state)
{
    moveit_msgs::DisplayRobotState newState;
    currentState_.state.joint_state.position = std::vector<double>(&state[0], &state[PANDA_NUM_JOINTS-1]);
    currentState_.state.joint_state.velocity = std::vector<double>(&state[PANDA_NUM_JOINTS], &state[2 * PANDA_NUM_JOINTS - 1]);
    currentState_.state.joint_state.effort = std::vector<double>(PANDA_NUM_JOINTS, 0.0);
    robot_state_publisher_.publish(currentState_);
}

void PandaMoveitInterface::sendControlCommand(std::vector<double> controlCommand)
{
    currentState_.state.joint_state.effort = controlCommand;

    robot_state_publisher_.publish(currentState_);
}

std::vector<double> PandaMoveitInterface::getRandomConfig(void)
{
    Eigen::VectorXd q;
    kinematic_state_->setToRandomPositions(joint_model_group_);
    kinematic_state_->copyJointGroupPositions(joint_model_group_, q);
    
    return std::vector<double>(q.data(), q.data() + q.size());
}
