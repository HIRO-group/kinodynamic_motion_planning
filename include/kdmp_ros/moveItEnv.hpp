#include <urdf/model.h>
#include <string>

#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/PlanningScene.h>

#include <shape_msgs/Mesh.h>
#include <shape_msgs/MeshTriangle.h>
#include <shape_msgs/Plane.h>
#include <shape_msgs/SolidPrimitive.h>
#include <ros/ros.h>

class MoveItEnv
{
    public:
        MoveItEnv(ros::NodeHandle nh, const std::string &base_frame);
        bool loadUrdf(std::string urdfPath);
        void urdfToRosMsg(std::string urdfPath, moveit_msgs::CollisionObject &obj);
        moveit_msgs::CollisionObject makeBox(const std::vector<double> &dims, const std::vector<double> &pose);
        moveit_msgs::CollisionObject makePlane(const std::vector<double> &coef, const std::vector<double> &pose);
        void createHiroScene(const std::string &base_frame);



    private:
        ros::NodeHandle nh_;
        urdf::Model model_;
        ros::Publisher planning_scene_diff_publisher_;
        moveit_msgs::PlanningScene env_;
        ros::WallDuration sleep_t_;
        std::string base_frame_;
        int itemNum_ = 0;
};