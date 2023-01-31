#include "kdmp_ros/pandaMoveitInterface.hpp"
#include "Eigen/Core"

PandaMoveitInterface::PandaMoveitInterface(ros::NodeHandle nh) : nh_(nh)
{
    ik_service_client_ = nh_.serviceClient<moveit_msgs::GetPositionIK>("compute_ik");
    robot_state_publisher_ =
      nh_.advertise<moveit_msgs::DisplayRobotState>("tutorial_robot_state", 1);

    while (!ik_service_client_.exists())
    {
        ROS_INFO("Waiting for service");
        ros::Duration(1.0).sleep();
    }
}

bool PandaMoveitInterface::inCollision(std::vector<double> q)
{
    return false;
}

std::vector<double> PandaMoveitInterface::inverseKinematics(std::vector<double> eePose)
{
    moveit_msgs::GetPositionIK::Request service_request;
    moveit_msgs::GetPositionIK::Response service_response;

    Eigen::Quaternionf quat = Eigen::AngleAxisf(eePose[3], Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(eePose[4], Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(eePose[5], Eigen::Vector3f::UnitZ());
    

    service_request.ik_request.group_name = "panda_arm";
    service_request.ik_request.pose_stamped.header.frame_id = "torso_lift_link";
    service_request.ik_request.pose_stamped.pose.position.x = eePose[0];
    service_request.ik_request.pose_stamped.pose.position.y = eePose[1];
    service_request.ik_request.pose_stamped.pose.position.z = eePose[2];

    service_request.ik_request.pose_stamped.pose.orientation.x = quat.x();
    service_request.ik_request.pose_stamped.pose.orientation.y = quat.y();
    service_request.ik_request.pose_stamped.pose.orientation.z = quat.z();
    service_request.ik_request.pose_stamped.pose.orientation.w = quat.w();

    service_request.ik_request.avoid_collisions = true;
    ik_service_client_.call(service_request, service_response);

    ROS_INFO_STREAM(
      "Result: " << ((service_response.error_code.val == service_response.error_code.SUCCESS) ? "True " : "False ")
                 << service_response.error_code.val);

    return service_response.solution.joint_state.position;
}

void PandaMoveitInterface::setRobotState(std::vector<double> state)
{
    //no op
}

void PandaMoveitInterface::sendControlCommand(std::vector<double> controlCommand)
{
    // no op
}