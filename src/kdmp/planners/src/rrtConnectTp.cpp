#include "kdmp/planners/include/rrtConnectTp.hpp"
#include "kdmp/spaces/include/pandaConstants.hpp"

#include "ompl/base/Planner.h"
#include <jacobi/jacobi.hpp>
#include <jacobi/robot.hpp>

RRTConnectTimePara::RRTConnectTimePara()
{}

Trajectory RRTConnectTimePara::solve(std::shared_ptr<omplBase::ProblemDefinition> problem,
            std::shared_ptr<omplBase::SpaceInformation> space)
{
    mSpace = space;
    mProblem = problem;

    findStaticTrajectory();
    performTimeParameterization();

    return mTrajectory;
}

void RRTConnectTimePara::findStaticTrajectory()
{
    configPlanner = std::make_shared<omplGeo::RRTConnect>(mSpace);
    configPlanner->setProblemDefinition(mProblem);
    configPlanner->setup();

    // attempt to solve the problem within one second of planning time
    omplBase::PlannerStatus solved = configPlanner->omplBase::Planner::solve(MAX_PLAN_TIME);

    if (solved)
    {
        // get the goal representation from the problem definition (not the same as the goal state)
        // and inquire about the found path
        omplBase::PathPtr path = mProblem->getSolutionPath();
        std::cout << "Found solution:" << std::endl;
        mConfigurations = path->as< std::vector<std::vector<double>> >();
    } else {
        mConfigurations = {};
    }
}

void RRTConnectTimePara::performTimeParameterization()
{
    if (mConfigurations.empty()) {
        mTrajectory = Trajectory();
        return;
    }
    
    

}