
#include <kdmp/problems/include/pandaSetup.hpp>
#include <kdmp/spaces/include/pandaStateSpace.hpp>
#include <kdmp/spaces/include/pandaControlSpace.hpp>
#include <kdmp/spaces/include/pandaStatePropogator.hpp>
#include <kdmp/spaces/include/pandaDirectedControlSampler.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/sst/SST.h>

namespace omplBase = ompl::base;
namespace omplBase = ompl::control;

/// @cond IGNORE
class PandaStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
    PandaStateValidityChecker(const ompl::base::SpaceInformationPtr &si, RobotInterface *rbt) : 
    ompl::base::StateValidityChecker(si), panda(rbt)
    {
    }

    bool isValid(const omplBase::State *state) const override
    {
        return si_->satisfiesBounds(state);
    }
    std::shared_ptr<RobotInterface> panda;
};
/// @endcond

ompl::control::DirectedControlSamplerPtr PandaDirectedControlSamplerAllocator(
    const omplControl::SpaceInformation *si, const omplBase::GoalPtr &goal, bool propogateMax)
{
    return std::make_shared<PandaDirectedControlSampler>(si, goal, propogateMax);
}

PandaSetup::PandaSetup(const std::string &plannerName, RobotInterface *robot, const std::vector<double> &stateVec)
    : omplControl::SimpleSetup(std::make_shared<PandaControlSpace>(TORQUE_CTL)), panda_(robot)
{
    const omplBase::StateSpacePtr &space = getStateSpace();
    space->setup();
    std::vector<double> goalVec(space->getDimension(), 0.0);
    omplBase::ScopedState<> start(space);
    auto goal = std::make_shared<omplBase::ScopedState<>>(space);
    if (stateVec.size() == space->getDimension()) {
        space->copyFromReals(start.get(), stateVec);
    } else {
        std::vector<double> startVec(space->getDimension(), 0.);
        std::vector<double> world_pose_start = {
            rng_.uniformReal(0.15, 0.5),
            rng_.uniformReal(-0.5, 0.5),
            rng_.uniformReal(0.0, 0.5)
        };
        std::vector<double> world_pose_goal = {
            rng_.uniformReal(0.15, 0.5),
            rng_.uniformReal(-0.5, 0.5),
            rng_.uniformReal(0.0, 0.5)
        };
        std::vector<double> q_start = panda_->inverseKinematics(world_pose_start);
        for (int i = 0; i< q_start.size(); i++) {
            startVec[i] = q_start[i];
        }
        space->copyFromReals(start.get(), startVec);
    }
    std::vector<double> world_pose_goal = {
        rng_.uniformReal(0.15, 0.5),
        rng_.uniformReal(-0.5, 0.5),
        rng_.uniformReal(0.0, 0.5)
    };
    std::vector<double> q_goal = panda_->inverseKinematics(world_pose_goal);

    for (int i = 0; i< q_goal.size(); i++) {
        goalVec[i] = q_goal[i];
    }
    space->copyFromReals(goal->get(), goalVec);

    setStartState(start);
    setGoal(goal);
    
    const omplBase::GoalPtr& goal = getGoal();
    si_->setDirectedControlSamplerAllocator(
        [&goal](const omplControl::SpaceInformation *si)
        {
            return PandaDirectedControlSamplerAllocator(si, goal, false);
        });

    setPlanner(getConfiguredPlannerInstance(plannerName));
    setStateValidityChecker(std::make_shared<PandaStateValidityChecker>(si_, panda));
    setStatePropagator(std::make_shared<PandaStatePropogator>(si_));
}

omplBase::PlannerPtr PandaSetup::getConfiguredPlannerInstance(const std::string& plannerName)
{
    if (plannerName == "est")
        return std::make_shared<omplControl::EST>(si_);
    if (plannerName == "kpiece")
        return std::make_shared<omplControl::KPIECE1>(si_);
    if (plannerName == "sst")
    {
        auto sstplanner(std::make_shared<omplControl::SST>(si_));
        sstplanner->setSelectionRadius(0.05);
        sstplanner->setPruningRadius(0.001);
        return sstplanner;
    }
    auto rrtplanner(std::make_shared<omplControl::RRT>(si_));
    rrtplanner->setIntermediateStates(true);
    return rrtplanner;
}
