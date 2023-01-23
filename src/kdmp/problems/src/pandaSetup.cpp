
#include <kdmp/problems/include/pandaSetup.hpp>
#include <kdmp/spaces/include/pandaStateSpace.hpp>
#include <kdmp/spaces/include/pandaControlSpace.hpp>
#include <kdmp/spaces/include/pandaDirectedControlSampler.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/kpiece/KPIECE1.h>
#include <ompl/control/planners/est/EST.h>
#include <ompl/control/planners/pdst/PDST.h>
#include <ompl/control/planners/sst/SST.h>

namespace omplBase = ompl::base;
namespace omplBase = ompl::control;

/// @cond IGNORE
class PandaStateValidityChecker : public ompl::base::StateValidityChecker
{
public:
    KoulesStateValidityChecker(const ompl::base::SpaceInformationPtr &si) : ompl::base::StateValidityChecker(si)
    {
    }

    bool isValid(const omplBase::State *state) const override
    {
        return si_->satisfiesBounds(state);
    }
};
/// @endcond

ompl::control::DirectedControlSamplerPtr PandaDirectedControlSamplerAllocator(
    const omplControl::SpaceInformation *si, const omplBase::GoalPtr &goal, bool propogateMax)
{
    return std::make_shared<PandaDirectedControlSampler>(si, goal, propogateMax);
}

PandaSetup::PandaSetup(std::string &plannerName, std::vector<double> &stateVec) :
    : omplControl::SimpleSetup(std::make_shared<PandaStateSpace>())
{
    const omplBase::StateSpacePtr &space = getStateSpace();
    space->setup();

    ob::ScopedState<> start(space);
    if (stateVec.size() == space->getDimension()) {
        space->copyFromReals(start.get(), stateVec);
    } else {
        std::vector<double> startVec(space->getDimension(), 0.);
        std::vector<double> q;
    }


    if (stateVec.empty()) {
        while ()
    }
}


