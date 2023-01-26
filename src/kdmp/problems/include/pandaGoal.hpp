#ifndef PANDA_GOAL_
#define PANDA_GOAL_

#include <ompl/base/goals/GoalLazySamples.h>

class PandaGoal : public ompl::base::GoalLazySamples
{
    public:
        PandaGoal(const ompl::base::SpaceInformationPtr &si)
            : ompl::base::GoalLazySamples(si), stateSampler_(si->allocStateSampler())
        {
            threshold_ = 0.001;
        }

        virtual double distanceGoal(const ompl::base::State *st) const;

        virtual void sampleGoal(ompl::base::State *st) const;

        virtual unsigned int maxSampleCount() const
        {
            return std::numeric_limits<unsigned int>::max();
        }
    
    private:
        ompl::RNG rng_;
        ompl::base::StateSamplerPtr stateSampler_;
        ompl::base::State *goal_ = nullptr;
};

#endif