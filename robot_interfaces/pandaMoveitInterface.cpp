#include "kdmp_ros/pandaMoveitInterface.hpp"
#include "franka_model.h"
#include "pandaConstants.hpp"
#include "rigidBodyDynamics.hpp"
#include "Eigen/Core"
#include "Eigen/LU"
#include "moveit_msgs/ApplyPlanningScene.h"

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
    kinematic_state_->setToRandomPositions(joint_model_group_, rng_);
    kinematic_state_->copyJointGroupPositions(joint_model_group_,
                                           currentState_.state.joint_state.position);

    ik_service_client_ = nh_.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    fk_service_client_ = nh.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");
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
    state.setJointGroupPositions("panda_arm", q);
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

    kinematic_state_->setToRandomPositions(joint_model_group_, rng_);
    kinematic_state_->copyJointGroupPositions(joint_model_group_, 
            service_request.ik_request.robot_state.joint_state.position);

    service_request.ik_request.avoid_collisions = true;
    ik_service_client_.call(service_request, service_response);

    ROS_INFO_STREAM(
      "Result: " << ((service_response.error_code.val == service_response.error_code.SUCCESS) ? "True " : "False ")
                 << service_response.error_code.val);
    if (not (service_response.error_code.val == service_response.error_code.SUCCESS)) {
        return std::vector<double>();
    }
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
    kinematic_state_->setToRandomPositions(joint_model_group_, rng_);
    kinematic_state_->copyJointGroupPositions(joint_model_group_, q);
    
    return Eigen::eigenVecTovVec(q);
}

std::vector<double> PandaMoveitInterface::forwardKinematics(std::vector<double> q)
{
    kinematic_state_->setJointGroupPositions(joint_model_group_, q);
    
    moveit_msgs::GetPositionFK::Request service_request;
    moveit_msgs::GetPositionFK::Response service_response;
    std::cout<<"in forward kinematics\n";
    service_request.fk_link_names = joint_model_group_->getLinkModelNames();
    service_request.robot_state.joint_state.position = q;
    service_request.header.frame_id = "panda_link0";
    std::cout<<"calling fk \n";
    fk_service_client_.call(service_request, service_response);
    std::cout<<"called fk \n";
    geometry_msgs::PoseStamped pose = *(service_response.pose_stamped.end() - 1);
    std::cout<<"Pose stamped \n";
    Eigen::Quaterniond quat = {pose.pose.orientation.x, pose.pose.orientation.y, pose.pose.orientation.z, pose.pose.orientation.w};

    auto euler = quat.toRotationMatrix().eulerAngles(0, 1, 2);


    return std::vector<double>({pose.pose.position.x, pose.pose.position.y, pose.pose.position.z,
            euler.x(), euler.y(), euler.z()});
    
}

std::vector<double> PandaMoveitInterface::eeVelToJointVel(std::vector<double> eeVel, std::vector<double> q)
{
    kinematic_state_->setJointGroupPositions(joint_model_group_, q);
    Eigen::MatrixXd J =  kinematic_state_->getJacobian(joint_model_group_);
    Eigen::Matrix<double, 6, 1> v;
    for (int i = 0; i < 6; i++) {
        v[i] = eeVel[i];
    }

    Eigen::Vector7d qd = J.inverse() * v;

    return std::vector<double>(qd.data(), qd.data() + qd.size());
    
}

std::vector<double> PandaMoveitInterface::jointVelToEeVel(std::vector<double> qd, std::vector<double> q)
{
    kinematic_state_->setJointGroupPositions(joint_model_group_, q);
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd J;
    kinematic_state_->getJacobian(joint_model_group_,
                             kinematic_state_->getLinkModel(joint_model_group_->getLinkModelNames().back()),
                             reference_point_position, J);
    Eigen::Vector7d v;
    for (int i = 0; i < PANDA_NUM_JOINTS; i++) {
        v[i] = qd[i];
    }
    Eigen::Matrix<double, 6, 1> eeVel = (J * v);
    ROS_INFO_STREAM("eeVel: \n" << eeVel << "\n");
    
    return Eigen::eigenVecTovVec(eeVel);
}

 Eigen::MatrixXd PandaMoveitInterface::getJacobian(std::vector<double> q)
 {
    kinematic_state_->setJointGroupPositions(joint_model_group_, q);
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd J;
    kinematic_state_->getJacobian(joint_model_group_,
                             kinematic_state_->getLinkModel(joint_model_group_->getLinkModelNames().back()),
                             reference_point_position, J);
    return J;
 }

 std::vector<double> PandaMoveitInterface::ddqFromEEAcc(std::vector<double> ee_acc, std::vector<double> dq, std::vector<double> q)
 {
    Eigen::Matrix<double, 6, 7> J = getJacobian(q).block<6,7>(0,0);
    Eigen::Matrix<double, 6, 7> Jdot = get_jacobian_derivative(J, dq);
    Eigen::Matrix<double, 6, 1> ee_aack = Eigen::vecToEigenVec(ee_acc);
    Eigen::Vector7d dqE = Eigen::vecToEigenVec(dq);
    Eigen::Vector7d ddq = J.transpose() * (ee_aack - Jdot * dqE);
    return Eigen::eigenVecTovVec(ddq);
 }