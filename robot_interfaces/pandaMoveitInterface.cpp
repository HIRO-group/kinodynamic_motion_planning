#include "kdmp_ros/pandaMoveitInterface.hpp"
#include "kdmp_ros/panda_ik.hpp"
#include "franka_model.h"
#include "pandaConstants.hpp"
#include "rigidBodyDynamics.hpp"
#include "Eigen/Core"
#include "Eigen/LU"
#include <array>
#include <algorithm>
#include "moveit_msgs/ApplyPlanningScene.h"
#include "geometry_msgs/Twist.h"

#include <math.h>

PandaMoveitInterface::PandaMoveitInterface(ros::NodeHandle nh) 
    : nh_(nh), robot_model_(moveit::core::loadTestingRobotModel("panda")), robot_model_loader_("robot_description"), control_queue_limit_(1000)
{
    ROS_DEBUG("initializing panda moveit interface");

    kinematic_model_ = robot_model_loader_.getModel();
    kinematic_state_ = std::make_shared<moveit::core::RobotState>(kinematic_model_);
    joint_model_group_ = kinematic_model_->getJointModelGroup("panda_arm");
    joint_model_group_hand_ = kinematic_model_->getJointModelGroup("panda_manipulator");
    
    currentState_.state.joint_state.name = joint_model_group_->getJointModelNames();
    kinematic_state_->setToRandomPositions(joint_model_group_, rng_);
    kinematic_state_->copyJointGroupPositions(joint_model_group_,
                                           currentState_.state.joint_state.position);

    ik_service_client_ = nh_.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    fk_service_client_ = nh.serviceClient<moveit_msgs::GetPositionFK>("compute_fk");
    robot_state_publisher_ = nh_.advertise<moveit_msgs::DisplayRobotState>("robot_state", 1);
    // control_sample_sub_ = nh_.subscribe("panda_data", 1000, &PandaMoveitInterface::controlCallback, this);
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
    
    // quat to rotation matrix
    Eigen::Matrix3f R = quat.normalized().toRotationMatrix();

    float *Rf = R.data();

    
    geometry_msgs::Pose pose;
    pose.position.x = eePose[0];
    pose.position.y = eePose[1];
    pose.position.z = eePose[2];
    pose.orientation.x =  quat.x();
    pose.orientation.y =  quat.y();
    pose.orientation.z =  quat.z();
    pose.orientation.w =  quat.w();

    kinematic_state_->setToRandomPositions();
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
    Eigen::Matrix<double, 6, 7> J = getJacobian(q);
    Eigen::Matrix<double, 6, 1> v;
    for (int i = 0; i < 6; i++) {
        v[i] = eeVel[i];
    }
    ROS_ERROR_STREAM("VVVV: " << v);
    Eigen::Matrix<double, 7, 6> J_inv; //= J.transpose();
    ROS_ERROR_STREAM("JACOBIAN full:");
    ROS_ERROR_STREAM(""<<getJacobian(q));

    ROS_ERROR_STREAM("JACOBIAN:");
    ROS_ERROR_STREAM(""<<J);
    
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(J.transpose());
    J_inv.setIdentity(7, 6);
    J_inv = qr.householderQ() * J_inv;  
    J_inv = qr.matrixQR().topLeftCorner(7, 6).triangularView<Eigen::Upper>().transpose().solve<Eigen::OnTheRight>(J_inv);
    ROS_ERROR_STREAM("Jacobian Inverse: " << J_inv);
    Eigen::Vector7d qd = J_inv * v;

    return std::vector<double>(qd.data(), qd.data() + qd.size());
}

std::vector<double> PandaMoveitInterface::jointVelToEeVel(std::vector<double> qd, std::vector<double> q)
{
    kinematic_state_->setJointGroupPositions(joint_model_group_, q);
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::Matrix<double, 6, 7> J = getJacobian(q);
    Eigen::Vector7d v;
    for (int i = 0; i < PANDA_NUM_MOVABLE_JOINTS; i++) {
        v[i] = qd[i];
    }
    Eigen::Matrix<double, 6, 1> eeVel = (J * v);
    // ROS_INFO_STREAM("eeVel: \n" << eeVel << "\n");
    
    return Eigen::eigenVecTovVec(eeVel);
}

 Eigen::Matrix4d get_tf_mat(int i, Eigen::Matrix<double, 10, 4> dh) 
 {
    double a = dh(i, 0), d = dh(i, 1);
    double alpha = dh(i, 2), theta = dh(i, 3);
    double q = theta;

    Eigen::Matrix4d aa;
    aa.setZero();
    aa(0, 0) = cos(q);
    aa(0, 1) = -sin(q);
    aa(0, 2) = 0;
    aa(0, 3) = a;
    aa(1, 0) = sin(q) * cos(alpha);
    aa(1, 1) = cos(q) * cos(alpha);
    aa(1, 2) = -sin(alpha);
    aa(1, 3) = -sin(alpha) * d;
    aa(2, 0) = sin(q) * sin(alpha);
    aa(2, 1) = cos(q) * sin(alpha);
    aa(2, 2) = cos(alpha);
    aa(2, 3) = cos(alpha) * d;
    aa(3, 0) = 0;
    aa(3, 1) = 0;
    aa(3, 2) = 0;
    aa(3, 3) = 1.0;

    return aa;
 }
 
 Eigen::MatrixXd PandaMoveitInterface::getJacobian(std::vector<double> joint_angles)
 {
    // Eigen::Matrix<double, 10, 4> dh_params;
    // dh_params.block(0, 0, 1, 4) = Eigen::Vector4d({0, 0.333, 0, joint_angles[0]});
    // dh_params.block(1,  0, 1, 4) = Eigen::Vector4d({0, 0, -M_PI_2, joint_angles[1]});
    // dh_params.block(2,  0, 1, 4) = Eigen::Vector4d({0, 0.316, M_PI_2, joint_angles[2]});
    // dh_params.block(3,  0, 1, 4) = Eigen::Vector4d({0.0825, 0, M_PI_2, joint_angles[3]});
    // dh_params.block(4, 0, 1, 4) =  Eigen::Vector4d({-0.0825, 0.384, -M_PI_2, joint_angles[4]});
    // dh_params.block(5, 0, 1, 4) = Eigen::Vector4d({0, 0, M_PI_2, joint_angles[5]});
    // dh_params.block(6, 0, 1, 4) = Eigen::Vector4d({0.088, 0, M_PI_2, joint_angles[6]});
    // dh_params.block(7, 0, 1, 4) = Eigen::Vector4d({0, 0.107, 0, 0});
    // dh_params.block(8, 0, 1, 4) = Eigen::Vector4d({0, 0, 0, -M_PI_4});
    // dh_params.block(9, 0, 1, 4) = Eigen::Vector4d({0.0, 0.1034, 0, 0});

    // Eigen::Matrix4d TT;
    // TT.setIdentity();

    // for (int i = 0; i < 10; i++) {
    //     TT = TT * get_tf_mat(i, dh_params);
    // }

    // Eigen::Matrix<double, 6, 10> J;
    // J.setZero();
    // Eigen::Matrix4d T;
    // T.setIdentity();

    // for (int i = 0; i < 10; i++) {
    //     T = T * get_tf_mat(i, dh_params);

    //     Eigen::Matrix<double, 3, 1> p = TT.block(0, 3, 3, 1) - T.block(0, 3, 3, 1);
    //     Eigen::Matrix<double, 3, 1> z = T.block(0, 2, 3, 1);
    //     J.block(0, i, 3, 1) = z.cross(p);
    //     J.block(3, i, 3, 1) = z;
    // }
   
    // return J.block(0, 0, 6, 7);
    ROS_ERROR_STREAM("TEST 1");
    kinematic_state_->setJointGroupPositions(joint_model_group_, Eigen::vecToEigenVec(joint_angles));
    ROS_ERROR_STREAM("TEST 2: " << joint_model_group_hand_->getLinkModelNames().back());
    Eigen::Vector3d reference_point_position(0.0, 0.0, 0.0);
    Eigen::MatrixXd jacobian;
    kinematic_state_->getJacobian(joint_model_group_,
                                kinematic_state_->getLinkModel(joint_model_group_hand_->getLinkModelNames()[joint_model_group_hand_->getLinkModelNames().size() - 2]),
                                reference_point_position, jacobian);
    ROS_ERROR_STREAM("Fn jacobian: "<<jacobian);

    return jacobian.block(0, 0, 6, 7);
 }

 std::vector<double> PandaMoveitInterface::ddqFromEEAcc(std::vector<double> ee_acc, std::vector<double> dq, std::vector<double> q)
 {
    Eigen::Matrix<double, 6, 7> J = getJacobian(q);
    Eigen::Matrix<double, 6, 7> Jdot = get_jacobian_derivative(J, dq);
    Eigen::Matrix<double, 6, 1> ee_aack = Eigen::vecToEigenVec(ee_acc);
    Eigen::Vector7d dqE = Eigen::vecToEigenVec(dq);
    printVec(Eigen::eigenVecTovVec(dqE), "Converted eigen: ");
    Eigen::Matrix<double, 7, 6> J_inv; //= J.inverse();

    // this is supposed to be the pseudo inv
    Eigen::HouseholderQR<Eigen::MatrixXd> qr(J.transpose());
    J_inv.setIdentity();
    J_inv = qr.householderQ() * J_inv;  
    J_inv = qr.matrixQR().topLeftCorner(7, 6).triangularView<Eigen::Upper>().transpose().solve<Eigen::OnTheRight>(J_inv);

    Eigen::Vector7d ddq = J_inv * (ee_aack - Jdot * dqE);
    return Eigen::eigenVecTovVec(ddq);
 }

 std::vector<double> PandaMoveitInterface::sampleControl()
 {
    std::lock_guard<std::mutex> lk(control_queue_mutex_);
    kdmp_ros::PandaControlCmd cmd = control_queue_.front();
    control_queue_.pop();
    std::vector<double> motionCmd = {
        cmd.cmd.linear.x, cmd.cmd.linear.y, cmd.cmd.linear.z,
        cmd.cmd.angular.x, cmd.cmd.angular.y, cmd.cmd.angular.z
    };
    return motionCmd;
 }

 std::vector<double> PandaMoveitInterface::sampleControlTest(kdmp_ros::PandaControlCmd &cmd)
 {
    ros::Rate rate(5);
    while (control_queue_.empty()) {
        ROS_ERROR("NO DATA IN CONTROL QUEUE");
        rate.sleep();
    }
     std::lock_guard<std::mutex> lk(control_queue_mutex_);
    cmd = control_queue_.front();
    control_queue_.pop();

    std::vector<double> motionCmd = {
        cmd.cmd.linear.x, cmd.cmd.linear.y, cmd.cmd.linear.z,
        cmd.cmd.angular.x, cmd.cmd.angular.y, cmd.cmd.angular.z
    };
    return motionCmd;
 }

//  void PandaMoveitInterface::controlCallback(const kdmp_ros::PandaControlCmd::ConstPtr &msg)
//  {
//     ROS_ERROR("HANDLING CONTROL DATA");
//     if (control_queue_.size() >= control_queue_limit_)
//         control_queue_.pop();
//     control_queue_.push(*msg);
//  }