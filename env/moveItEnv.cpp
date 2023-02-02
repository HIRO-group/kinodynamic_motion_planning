#include "kdmp_ros/moveItEnv.hpp"
#include <Eigen/Core>
#include <tf2_eigen/tf2_eigen.h>


MoveItEnv::MoveItEnv(ros::NodeHandle nh, const std::string &base_frame)
    : nh_(nh), sleep_t_(0.5), base_frame_(base_frame)
{
    planning_scene_diff_publisher_ =
            nh_.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
    
    while(planning_scene_diff_publisher_.getNumSubscribers() < 1) {
        sleep_t_.sleep();
    }
}


void MoveItEnv::createHiroScene(const std::string &base_frame)
{
    const std::vector<double> box1_dims({0.9144, 1.825, 0.95});
    const std::vector<double> box2_dims({0.8001, 1.6002, 0.857});
    const std::vector<double> box3_dims({0.1, 2.0, 2.0});

    

    Eigen::Quaternionf q = Eigen::AngleAxisf(0, Eigen::Vector3f::UnitX())
    * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitY())
    * Eigen::AngleAxisf(0, Eigen::Vector3f::UnitZ());
     
    const std::vector<double> box1_pose({-0.2994, 0.0, -0.5131, q.x(), q.y(), q.z(), q.w()});
    const std::vector<double> box2_pose({0.6218, 0.0, -0.53645, q.x(), q.y(), q.z(), q.w()});
    const std::vector<double> box3_pose({-0.7366, 0, 0, q.x(), q.y(), q.z(), q.w()});

    const std::vector<double> plane_coefs({0, 0, 1.0, -1.0});
    const std::vector<double> plane_pose({0.0, 0.0, 0.0, q.x(), q.y(), q.z(), q.w()});

    auto box1 = makeBox(box1_dims, box1_pose);
    auto box2 = makeBox(box2_dims, box2_pose);
    auto box3 = makeBox(box3_dims, box3_pose);
    auto floor = makePlane(plane_coefs, plane_pose);


    moveit_msgs::PlanningScene planning_scene;
    planning_scene.world.collision_objects.push_back(box1);
    planning_scene.world.collision_objects.push_back(box2);
    planning_scene.world.collision_objects.push_back(box3);
    planning_scene.world.collision_objects.push_back(floor);

    planning_scene.is_diff = true;

    env_ = planning_scene;

    planning_scene_diff_publisher_.publish(planning_scene);    
}

bool MoveItEnv::loadUrdf(std::string urdfPath)
{
    model_.initFile(urdfPath);
    int type = model_.links_[0].get()->collision_array[0]->geometry->type == urdf::Geometry::SPHERE;
}

moveit_msgs::CollisionObject MoveItEnv::makeBox(const std::vector<double> &dims, const std::vector<double> &pose)
{
    moveit_msgs::CollisionObject obj;
    shape_msgs::SolidPrimitive box;
    geometry_msgs::Pose box_pose;
    box.type = box.BOX;
    box.dimensions.resize(3);
    box.dimensions[box.BOX_X] = dims[box.BOX_X];
    box.dimensions[box.BOX_Y] = dims[box.BOX_Y];
    box.dimensions[box.BOX_Z] = dims[box.BOX_Z];

    box_pose.position.x = pose[0];
    box_pose.position.y = pose[1];
    box_pose.position.z = pose[2];
    box_pose.orientation.x = pose[3];
    box_pose.orientation.y = pose[4];
    box_pose.orientation.z = pose[5];
    box_pose.orientation.w = pose[6];

    obj.operation = obj.ADD;
    obj.header.frame_id = base_frame_;
    obj.id = "Box"+std::to_string(itemNum_++);


    obj.primitives.push_back(box);
    obj.primitive_poses.push_back(box_pose);
    return obj;
}

moveit_msgs::CollisionObject MoveItEnv::makePlane(const std::vector<double> &coef, const std::vector<double> &pose)
{
    moveit_msgs::CollisionObject obj;
    shape_msgs::Plane plane;
    geometry_msgs::Pose plane_pose;
    plane.coef[0] = coef[0];
    plane.coef[1] = coef[1];
    plane.coef[2] = coef[2];
    plane.coef[3] = coef[3];

    plane_pose.position.x = pose[0];
    plane_pose.position.y = pose[1];
    plane_pose.position.z = pose[2];

    plane_pose.orientation.x = pose[3];
    plane_pose.orientation.y = pose[4];
    plane_pose.orientation.z = pose[5];
    plane_pose.orientation.w = pose[6];

    obj.operation = obj.ADD;
    obj.header.frame_id = base_frame_;
    obj.id = "Plane";
    obj.header.frame_id = base_frame_;



    obj.planes.push_back(plane);
    obj.plane_poses.push_back(plane_pose);

    return obj;
}

void MoveItEnv::urdfToRosMsg(std::string urdfPath, moveit_msgs::CollisionObject &obj)
{
    model_.initFile(urdfPath);
    for (const auto &pair : model_.links_)
    {
        shape_msgs::SolidPrimitive prim;
        shape_msgs::Mesh mesh;
        const auto link = pair.second;
        int type = link->collision->geometry->type;
        if(type == urdf::Geometry::BOX) {
            prim.type = prim.BOX;
            urdf::Box *geo = dynamic_cast<urdf::Box *>(link->collision->geometry.get());
            prim.dimensions.resize(3);
            prim.dimensions[1] = geo->dim.x;
            prim.dimensions[1] = geo->dim.y;
            prim.dimensions[1] = geo->dim.z;
        } else if (type == urdf::Geometry::SPHERE) {
            prim.type = prim.SPHERE;
            urdf::Sphere *geo = dynamic_cast<urdf::Sphere *>(link->collision->geometry.get());
            prim.dimensions.resize(1);
            prim.dimensions[0] = geo->radius;
        } else if (type == urdf::Geometry::CYLINDER) {
            prim.type = prim.CYLINDER;
            urdf::Cylinder *geo = dynamic_cast<urdf::Cylinder *>(link->collision->geometry.get());
            prim.dimensions.resize(2);
            prim.dimensions[0] = geo->length;
            prim.dimensions[1] = geo->radius;
        }
    }
}