#ifndef PANDA_GOAL_
#define PANDA_GOAL_

#include <ompl/base/goals/GoalLazySamples.h>
#include <kdmp/problems/include/RobotInterface.hpp>
#include <kdmp/spaces/include/pandaStateSpace.hpp>


#include <boost/math/constants/constants.hpp>

#ifndef PI
#define PI boost::math::constants::pi<double>()
#define TWOPI boost::math::constants::two_pi<double>()
#endif

class PandaGoal : public ompl::base::GoalLazySamples
{
    public:
        PandaGoal(const ompl::base::SpaceInformationPtr &si, std::shared_ptr<RobotInterface> rbt, const std::vector<double> &goalState, double threshold=0.001)
            : ompl::base::GoalLazySamples(
            si, [this](const ompl::base::GoalLazySamples *, ompl::base::State *st) { return sampleGoalThread(st); },
            true, threshold), stateSampler_(si->allocStateSampler()), goalState_(goalState), panda_(rbt)
        {
            threshold_ = 0.001;
        }

        virtual double distanceGoal(const ompl::base::State *st) const
        {
            const double *state = st->as<PandaStateSpace::StateType>()->values;
            double dist = 0;
            for (int i = 0; i < si_->getStateDimension(); i++) {
                dist += std::pow(goalState_[i] - state[i], 2);
            }
            return std::sqrt(dist);
        }

        virtual bool sampleGoalThread(ompl::base::State *st) const
        {
            std::vector<double> seed(si_->getStateDimension());

            bool good = false;
            unsigned int maxTries = 1000;
            unsigned int tries = 0; 
            auto bounds_pose = PANDA_JOINT_LIMS;
            auto bounds_vel = PANDA_VEL_LIMS;
            do
            {
                for (size_t i = 0; i < seed.size(); ++i){
                    if (i < PANDA_NUM_JOINTS) {
                        seed[i] = rng_.uniformReal(bounds_pose[i][0], bounds_pose[i][1]);
                    } else {
                        int index = i - PANDA_NUM_JOINTS;
                        seed[i] = 0;
                    }
                }

                for (size_t i = 0; i < 10 && !good; ++i)
                {
                    std::vector<double> pose(goalState_);
                    if (not panda_->inCollision(seed))
                    {
                        // copy values into state
                        memcpy(st->as<PandaStateSpace::StateType>()->values, &seed[0],
                            seed.size() * sizeof(double));
                        // GoalLazySamples will check validity
                        good = true;
                    }
                    tries++;
                }
            } while (not good and tries < maxTries);
            return good;
        }
    
    private:
        mutable ompl::RNG rng_;
        ompl::base::StateSamplerPtr stateSampler_;
        std::vector<double> goalState_;
        std::shared_ptr<RobotInterface> panda_;
};

#endif