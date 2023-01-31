#include <ros/ros.h>

// #include <kdmp/problems/include/RobotInterface.hpp>
// #include <kdmp_ros/pandaMoveitInterface.hpp>
// #include <kdmp/spaces/include/pandaConstants.hpp>
// #include "kdmp/problems/include/pandaSetup.hpp"
// #include <kdmp/problems/include/pandaGoal.hpp>

#include "pandaSetup.hpp"
#include "pandaConstants.hpp"
#include "pandaMoveitInterface.hpp"
#include "pandaGoal.hpp"
#include "moveItEnv.hpp"

#include <ompl/control/SimpleSetup.h>
#include <ompl/control/planners/rrt/RRT.h>

#include <string>
#include <vector>

void print_path(std::vector<ompl::base::State *> path)
{
    std::stringstream ss;
    int i = 0;
    for (const auto &state : path) {
        ss << "State "<< i++ << ":\n";
        ss << "q : ";
        for (int i = 0; i < PANDA_NUM_JOINTS; i++) {
            ss << state->as<PandaStateSpace::StateType>()->values[i] << " ";
        }
        ss << "\n" << "qdd : ";
        for (int i = PANDA_NUM_JOINTS; i < 2 * PANDA_NUM_JOINTS; i++) {
            ss << state->as<PandaStateSpace::StateType>()->values[i] << " ";
        }
        ss << "\n\n";
    }
    ROS_INFO(ss.str().c_str());
}


void SolveProblem(ompl::control::SimpleSetupPtr setup, double timeout, bool write_viz_out)
{
     ompl::base::PlannerPtr rrt(new ompl::control::RRT(setup->getSpaceInformation()));
     setup->setPlanner(rrt);

     ompl::base::PlannerStatus status = setup->solve(timeout);

     if (status == ompl::base::PlannerStatus::EXACT_SOLUTION ||
        status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
    {
        ompl::control::PathControl &pctl = setup->getSolutionPath();
        auto path = pctl.getStates();
        print_path(path);
        // OMPL_INFORM("Solution path has %d states", pgeo.getStateCount());

        // if (write_viz_out)
        // {
        //     pgeo.interpolate(250);
        //     WriteVisualization(problem, pgeo, xySlices);
        // }
    }
    else
    {
        ROS_WARN("Planning failed");
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ompl_rrt");

    ros::NodeHandle n;
    ROS_ERROR("STARTING");
    std::shared_ptr<RobotInterface> robot_interface = std::make_shared<PandaMoveitInterface>(n);
    ROS_ERROR("Created robot interface");
    std::vector<double> startVec(2 * PANDA_NUM_JOINTS, 0.0);
    std::string plannerType = "rrt";
    
    ompl::control::SimpleSetupPtr setup = std::make_shared<PandaSetup>(plannerType.c_str(), robot_interface, startVec);
    SolveProblem(setup, 5.0, false);
    return 0;
}