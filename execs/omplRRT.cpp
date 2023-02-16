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
#include <pandaControlSpace.hpp>

#include <ompl/control/planners/rrt/RRT.h>

#include <string>
#include <cstdlib>
#include <unistd.h>
#include <vector>

void print_path(std::vector<ompl::base::State *> path,std::vector<ompl::control::Control *> ctls, std::vector<double> times, std::shared_ptr<RobotInterface> rbt)
{
    std::stringstream ss;
    for (int i = 0; i < path.size(); i++) {
        const auto state = path[i];
        
        ss << "State "<< i << ":\n";
        ss << "q : ";
        double *q = state->as<PandaStateSpace::StateType>()->values;
        std::vector<double> qVec(q, q + PANDA_NUM_JOINTS);
        for (int i = 0; i < PANDA_NUM_JOINTS; i++) {
            ss << state->as<PandaStateSpace::StateType>()->values[i] << " ";
        }
        ss << "\neeState: " << vecToString(rbt->forwardKinematics(qVec));
        ss << "\n" << "qd : ";
        for (int i = PANDA_NUM_JOINTS; i < 2 * PANDA_NUM_JOINTS; i++) {
            ss << state->as<PandaStateSpace::StateType>()->values[i] << " ";
        }
        if (i < ctls.size()) {
            const auto ctl = ctls[i];
            double *ctlVals = ctl->as<PandaControlSpace::ControlType>()->values;
            std::vector<double> ctlVec(ctlVals, ctlVals + 6);
             ss << "\n" << "Commanded eevel: " << vecToString(ctlVec);
        }
       
        if (i < times.size()) {
            ss<< "\n" << "Duration: " << times[i];
        }
        
        ss << "\n\n";
    }
    ROS_INFO(ss.str().c_str());
}

void simPath(const std::vector<double> &times, const std::vector<ompl::control::Control *> &ctls,std::shared_ptr<RobotInterface> robot_interface )
{
    for (int i = 0; i < times.size(); i++) {
        double *cmd = ctls[i]->as<PandaControlSpace::ControlType>()->values;
        std::vector<double> command(cmd, cmd + PANDA_NUM_JOINTS);
        robot_interface->sendControlCommand(command);
         usleep(times[i] * 1000);
    }
}

void SolveProblem(ompl::control::SimpleSetupPtr setup, double timeout, bool write_viz_out, std::shared_ptr<RobotInterface> robot_interface)
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
        
        print_path(path, ctls, times, robot_interface);
        auto pandaGoal = setup->getGoal()->as<PandaGoal>();
        std::vector<double> goalVec(setup->getGoal()->as<PandaGoal>()->pubGoal_);
        ROS_INFO_STREAM("goalState: " << Eigen::vecToEigenVec( std::vector<double>(goalVec.begin(), goalVec.begin() + 6)));
        double *q = (*(path.end()-1))->as<PandaStateSpace::StateType>()->values;
        
        std::vector<double> qVec(q, q + PANDA_NUM_JOINTS);
        ROS_INFO_STREAM("End pose: " << Eigen::vecToEigenVec(robot_interface->forwardKinematics(qVec)));
        std::cout << "distance from goal: " << pandaGoal->distanceGoal( (*(path.end()-1)));
        simPath(times, ctls, robot_interface);
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

    SolveProblem(setup, 120, false, robot_interface);
    ros::shutdown();
    return 0;
}