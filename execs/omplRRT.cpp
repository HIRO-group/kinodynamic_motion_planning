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
#include "kdmpUtils.hpp"
#include "pandaControlSpace.hpp"
#include <ompl/control/PathControl.h>

#include <ompl/control/planners/rrt/RRT.h>

#include <string>
#include <vector>

void print_path(std::vector<ompl::base::State *> path,std::vector<ompl::control::Control *> ctls, std::vector<double> times)
{
    std::stringstream ss;
    for (int i = 0; i < path.size(); i++) {
        const auto state = path[i];
        
        ss << "State "<< i << ":\n";
        ss << "q : ";
        for (int i = 0; i < PANDA_NUM_JOINTS; i++) {
            ss << state->as<PandaStateSpace::StateType>()->values[i] << " ";
        }
        ss << "\n" << "qd : ";
        for (int i = PANDA_NUM_JOINTS; i < 2 * PANDA_NUM_JOINTS; i++) {
            ss << state->as<PandaStateSpace::StateType>()->values[i] << " ";
        }
        if (i < ctls.size()) {
            const auto ctl = ctls[i];
            double *ctlVals = ctl->as<PandaControlSpace::ControlType>()->values;
            std::vector<double> ctlVec(ctlVals, ctlVals + PANDA_NUM_MOVABLE_JOINTS);
             ss << "\n" << "Torques: " << vecToString(ctlVec);
        }
       
        if (i < times.size()) {
            ss<< "\n" << "Duration: " << times[i];
        }
        
        ss << "\n\n";
    }
    ROS_INFO(ss.str().c_str());
}


void SolveProblem(ompl::control::SimpleSetupPtr setup, double timeout, bool write_viz_out)
{
    ROS_ERROR("PLANNER SETUP");
    ompl::base::PlannerStatus status = setup->solve(timeout);
    ROS_INFO("Solver finished.");
    if (status == ompl::base::PlannerStatus::EXACT_SOLUTION ||
        status == ompl::base::PlannerStatus::APPROXIMATE_SOLUTION)
    {
        ompl::control::PathControl &pctl = setup->getSolutionPath();
        auto path = pctl.getStates();
        auto times = pctl.getControlDurations();
        auto ctls = pctl.getControls();
        print_path(path, ctls, times);
        ROS_INFO_STREAM("goalState: " << Eigen::vecToEigenVec( setup->getGoal()->as<PandaGoal>()->pubGoal_));
        double *q = path[path.size() - 1]->as<PandaStateSpace::StateType>()->values;
        ROS_INFO_STREAM("End pose: " << Eigen::vecToEigenVec(std::vector<double>(q, q + PANDA_NUM_JOINTS)));
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
    std::vector<double> startVec;
    std::string plannerType = "rrt";
    
    MoveItEnv env(n, "panda_link0");
    env.createHiroScene("");
    ompl::control::SimpleSetupPtr setup = std::make_shared<PandaSetup>(plannerType.c_str(), robot_interface, startVec);
    // PandaSetupSimple setup = PandaSetupSimple(plannerType.c_str(), robot_interface, startVec);

    SolveProblem(setup, 120, false);
    return 0;
}