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

    Eigen::Quaternionf quat = Eigen::AngleAxisf(eePose[3], Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(eePose[4], Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(eePose[5], Eigen::Vector3f::UnitZ());

    geometry_msgs::Pose pose;
    pose.position.x = eePose[0];
    pose.position.y = eePose[1];
    pose.position.z = eePose[2];
    pose.orientation.x =  quat.x();
    pose.orientation.y =  quat.y();
    pose.orientation.z =  quat.z();
    pose.orientation.w =  quat.w();


    kinematic_state_->setFromIK(joint_model_group_, pose, "panda_link8");

    Eigen::VectorXd jointPositions;
    kinematic_state_->copyJointGroupPositions(joint_model_group_, jointPositions);
    return Eigen::eigenVecTovVec(jointPositions);
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
    const Eigen::Affine3d &end_effector_state = kinematic_state_->getGlobalLinkTransform("panda_link8");
    auto trans =  end_effector_state.translation();
    auto rot = end_effector_state.rotation().eulerAngles(0, 1, 2);
    return std::vector<double>({trans.x(), trans.y(), trans.z(), rot.x(), rot.y(), rot.z()});
}

std::vector<double> PandaMoveitInterface::eeVelToJointVel(std::vector<double> eeVel, std::vector<double> q)
{
    kinematic_state_->setJointGroupPositions(joint_model_group_, q);
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::Matrix<double, 6, 7> J = getJacobian(q).block<6,7>(0,0);
    Eigen::Matrix<double, 6, 1> v;
    for (int i = 0; i < 6; i++) {
        v[i] = eeVel[i];
    }
    Eigen::Matrix<double, 7, 6> J_inv = J.transpose();

    // Eigen::HouseholderQR<Eigen::MatrixXd> qr(J.transpose());
    // J_inv.setIdentity(7, 6);
    // J_inv = qr.householderQ() * J_inv;  
    // J_inv = qr.matrixQR().topLeftCorner(7, 6).triangularView<Eigen::Upper>().transpose().solve<Eigen::OnTheRight>(J_inv);

    Eigen::Vector7d qd = J_inv * v;

    return std::vector<double>(qd.data(), qd.data() + qd.size());
}

std::vector<double> PandaMoveitInterface::jointVelToEeVel(std::vector<double> qd, std::vector<double> q)
{
    kinematic_state_->setJointGroupPositions(joint_model_group_, q);
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::Matrix<double, 6, 7> J = getJacobian(q).block<6,7>(0,0);
    Eigen::Vector7d v;
    for (int i = 0; i < PANDA_NUM_MOVABLE_JOINTS; i++) {
        v[i] = qd[i];
    }
    Eigen::Matrix<double, 6, 1> eeVel = (J * v);
    // ROS_INFO_STREAM("eeVel: \n" << eeVel << "\n");
    
    return Eigen::eigenVecTovVec(eeVel);
}

 Eigen::MatrixXd PandaMoveitInterface::getJacobian(std::vector<double> q)
 {
    kinematic_state_->setJointGroupPositions(joint_model_group_, q);
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    // std::cout<<"\n\n";
    // for (const std::string &name : joint_model_group_->getJointModelNames()) {
    //     std::cout<<name<<std::endl;
    // }
    // std::cout<<"\n\n";

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
    Eigen::Matrix<double, 7, 6> J_inv = J.transpose();

    // this is supposed to be the pseudo inv
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(J.transpose());
    // J_inv.setIdentity(7, 6);
    // J_inv = qr.householderQ() * J_inv;  
    // J_inv = qr.matrixQR().topLeftCorner(7, 6).triangularView<Eigen::Upper>().transpose().solve<Eigen::OnTheRight>(J_inv);

    Eigen::Vector7d ddq = J_inv * (ee_aack - Jdot * dqE);
    return Eigen::eigenVecTovVec(ddq);
 }