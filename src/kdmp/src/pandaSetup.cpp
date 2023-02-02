
#include <pandaSetup.hpp>
#include <pandaGoal.hpp>
#include <pandaStateSpace.hpp>
#include <pandaControlSpace.hpp>
#include <pandaStatePropogator.hpp>
#include <pandaDirectedControlSampler.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/sst/SST.h>
#include <iostream>
namespace omplBase = ompl::base;
namespace omplControl = ompl::control;

/// @cond IGNORE
class PandaStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
    PandaStateValidityChecker(const ompl::base::SpaceInformationPtr &si, std::shared_ptr<RobotInterface> rbt) : 
    ompl::base::StateValidityChecker(si), panda(rbt), si_(si)
    {
    }

    bool isValid(const omplBase::State *state) const override
    {
        
        double *stateVals = state->as<PandaStateSpace::StateType>()->values;
        std::vector<double> stateVec(stateVals, stateVals + si_->getStateDimension());
        return si_->satisfiesBounds(state) and not panda->inCollision(stateVec);
    }
    std::shared_ptr<RobotInterface> panda;
    const ompl::base::SpaceInformationPtr si_;
};
/// @endcond

ompl::control::DirectedControlSamplerPtr PandaDirectedControlSamplerAllocator(
    const omplControl::SpaceInformation *si, const omplBase::GoalPtr &goal, bool propogateMax)
{
    return std::make_shared<PandaDirectedControlSampler>(si, goal, propogateMax);
}

PandaSetup::PandaSetup(const char* plannerName, std::shared_ptr<RobotInterface> robot, std::vector<double> &stateVec)
    : omplControl::SimpleSetup(std::make_shared<PandaControlSpace>()), panda_(robot)
{
    std::cerr<<"in panda setup\n";
    const omplBase::StateSpacePtr &space = getStateSpace();
    std::cerr<<"got space ptr\n";
    space->setup();
    std::cerr<<"setup space\n";
    std::vector<double> goalVec(space->getDimension(), 0.0);
    omplBase::ScopedState<> start(space);
    // auto goal = std::make_shared<omplBase::ScopedState<>>(space);
    if (stateVec.size() == space->getDimension()) {
        space->copyFromReals(start.get(), stateVec);
    } else {
        std::vector<double> startVec = panda_->getRandomConfig();
        
        space->copyFromReals(start.get(), startVec);
    }
    
    std::vector<double> q_goal = panda_->getRandomConfig();

    for (int i = 0; i< q_goal.size(); i++) {
        goalVec[i] = q_goal[i];
    }

    setStartState(start);
    std::vector<double> goalState();
    setGoal(std::make_shared<PandaGoal>(si_, panda_, goalVec));
    double stepSize = 1.0 / PANDA_CTL_RATE; 
    si_->setPropagationStepSize(stepSize);
    si_->setMinMaxControlDuration(1, MAX_NUM_STEPS);
    
    const omplBase::GoalPtr& goal = getGoal();
    si_->setDirectedControlSamplerAllocator(
        [&goal](const omplControl::SpaceInformation *si)
        {
            return PandaDirectedControlSamplerAllocator(si, goal, false);
        });

    setPlanner(getConfiguredPlannerInstance(plannerName));
    setStateValidityChecker(std::make_shared<PandaStateValidityChecker>(si_, panda_));
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
