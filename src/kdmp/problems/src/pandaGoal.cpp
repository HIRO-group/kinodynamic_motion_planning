#include <kdmp/problems/include/pandaGoal.hpp>
#include <kdmp/spaces/include/pandaStateSpace.hpp>

double PandaGoal::distanceGoal(const ompl::base::State *st) const
{
    // if (not goal_) {
    //     sampleGoal(goal_);
    // }
    double *goalState = goal_->as<PandaStateSpace::StateType>()->values;
    double *curState = st->as<PandaStateSpace::StateType>()->values;
    int numDims = si_->getStateDimension();
    double sum = 0;
    for (int i = 0; i < numDims; i++) {
        sum += std::pow(goalState[i] - curState[i], 2);
    }
    return std::sqrt(sum);
}

void PandaGoal::sampleGoal(ompl::base::State *st) const
{
    std::vector<double> goal;
    
}