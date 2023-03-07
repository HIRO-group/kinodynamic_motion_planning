
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
#include <ompl/control/SimpleDirectedControlSampler.h>

#include <iostream>
#include <chrono>

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
        double *s = state->as<PandaStateSpace::StateType>()->values;
        std::vector<double> q(s, s + PANDA_NUM_MOVABLE_JOINTS);
        if ( not si_->satisfiesBounds(state)) {
            return false;
        }
        
        // printVec(q, "is valid q: ");
        if (q.empty()) {
            std::cout<<"\n EMPTY STATE \n";
            return false;
        } 
        bool isInCollision = panda->inCollision(q);

        return not isInCollision;
    }
    std::shared_ptr<RobotInterface> panda;
    const ompl::base::SpaceInformationPtr si_;
};
/// @endcond

ompl::control::DirectedControlSamplerPtr PandaDirectedControlSamplerAllocator(
    const omplControl::SpaceInformation *si, const omplBase::GoalPtr &goal, std::shared_ptr<RobotInterface> rbt)
{
    // return std::make_shared<ompl::control::SimpleDirectedControlSampler>(si, 10);
    return std::make_shared<PandaDirectedControlSampler>(si, goal, rbt);
}

PandaSetup::PandaSetup(const char* plannerName, std::shared_ptr<RobotInterface> robot, std::vector<double> &stateVec)
    : omplControl::SimpleSetup(std::make_shared<PandaControlSpace>()), panda_(robot)
{
    std::cerr<<"in panda setup\n";
    const omplBase::StateSpacePtr &space = getStateSpace();
    std::cerr<<"got space ptr\n";
    space->setup();
    setStateValidityChecker(std::make_shared<PandaStateValidityChecker>(si_, panda_));
    setStatePropagator(std::make_shared<PandaStatePropogator>(si_, panda_));
    std::cerr<<"setup space\n";
    omplBase::ScopedState<> start(space);
    if (stateVec.size() == space->getDimension()) {
        space->copyFromReals(start.get(), stateVec);
    } else {
        std::cout<<"startvec setting\n";
        std::vector<double> startVec = panda_->getRandomConfig();

        std::cout<<"startvec size: "<< startVec.size() << std::endl;
        for(int i = 0; i < PANDA_NUM_JOINTS; i++) {
            startVec.push_back(0.0);
        }
        std::cout<<"startvec set\n";
        space->copyFromReals(start.get(), startVec);
        std::cout<<"startvec copied to start state\n";
        while (not si_->isValid(start->as<PandaStateSpace::StateType>()))
        {
            startVec = panda_->getRandomConfig();
            for(int i = 0; i < PANDA_NUM_JOINTS; i++) {
                startVec.push_back(0.0);
            }
            space->copyFromReals(start.get(), startVec);
        }
    }
    omplBase::ScopedState<> goal_test(space);
    std::vector<double> goalVecTest = panda_->getRandomConfig();
    
    
    for (int i = 0; i < PANDA_NUM_JOINTS; i++) {
        goalVecTest.push_back(0.0);
    }
    space->copyFromReals(goal_test.get(), goalVecTest);
    while (not si_->isValid(goal_test->as<PandaStateSpace::StateType>()))
    {
        goalVecTest = panda_->getRandomConfig();
        for(int i = 0; i < PANDA_NUM_JOINTS; i++) {
            goalVecTest.push_back(0.0);
        }
        space->copyFromReals(goal_test.get(), goalVecTest);
    }
    
    std::vector<double> goalVec = panda_->forwardKinematics(goalVecTest);
    for (int i = 0; i < 6; i++) {
        goalVec.push_back(0.0);
    }
    std::cout<<"goal size: "<<goalVec.size()<<std::endl;
    
    setStartState(start);
    std::cout<<"set Start state"<<std::endl;
    setGoal(std::make_shared<PandaGoal>(si_, panda_, goalVec));
    std::cout<<"set goal\n";
    double stepSize = 1.0 / PANDA_CTL_RATE; 
    si_->setPropagationStepSize(stepSize);
    si_->setMinMaxControlDuration(1, MAX_NUM_STEPS);
    
    const omplBase::GoalPtr& goal = getGoal();
    si_->setDirectedControlSamplerAllocator(
        [&goal, robot](const omplControl::SpaceInformation *si)
        {
            return PandaDirectedControlSamplerAllocator(si, goal, robot);
        });

    setPlanner(getConfiguredPlannerInstance(plannerName));
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
