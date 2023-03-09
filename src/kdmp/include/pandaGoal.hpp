#ifndef PANDA_GOAL_
#define PANDA_GOAL_

#include <ompl/base/goals/GoalLazySamples.h>
#include <RobotInterface.hpp>
#include <pandaStateSpace.hpp>
#include "kdmpUtils.hpp"


#include <boost/math/constants/constants.hpp>
#include <iostream>
#include <algorithm>

#include "Eigen/Core"
#include <Eigen/LU>
#include <Eigen/Geometry>

#ifndef PI
#define PI boost::math::constants::pi<double>()
#define TWOPI boost::math::constants::two_pi<double>()
#endif

class PandaGoal : public ompl::base::GoalLazySamples
{
    public:
        PandaGoal(const ompl::base::SpaceInformationPtr &si, std::shared_ptr<RobotInterface> rbt, const std::vector<double> &goalState, double threshold=0.01)
            : ompl::base::GoalLazySamples(
            si, [this](const ompl::base::GoalLazySamples *, ompl::base::State *st) { return sampleGoalThread(st); },
            true), stateSampler_(si->allocStateSampler()), goalPose_(goalState), panda_(rbt), pubGoal_(goalState),
            goalQuat_(Eigen::AngleAxisf(goalState[3], Eigen::Vector3f::UnitX())
                * Eigen::AngleAxisf(goalState[4], Eigen::Vector3f::UnitY())
                * Eigen::AngleAxisf(goalState[5], Eigen::Vector3f::UnitZ()))
        {
            threshold_ = 0.01;
        }

        double distEuclid(double *st, double *goal, int rng) const
        {
            double dist = 0;
            for (int i = 0; i < rng; i++) {
                double diff = goal[i] - st[i];
                dist += diff * diff;
            }
            return sqrt(dist);
        }

        /**
         * @brief find the euler distance from st to the goal state
         * 
         * @param[in] st - state whose distance from goal is being checked
        */
        virtual double distanceGoal(const ompl::base::State *st) const
        {
            double dist = std::numeric_limits<double>::infinity();
            double *vals = st->as<PandaStateSpace::StateType>()->values;
            double *match = nullptr;
            for (auto state : states_)
            {
                double *goal = state->as<PandaStateSpace::StateType>()->values;
                double d = distEuclid(vals, goal, PANDA_NUM_JOINTS * 2);
                if (d < dist) {
                    dist = d;
                    match = goal;
                }
            }
            if (match) {
                printVec(std::vector<double>(match, match+PANDA_NUM_JOINTS), "MATCHED GOAL: ");
                printVec(std::vector<double>(vals, vals+PANDA_NUM_JOINTS), "TEST STATE: ");
                printVec(panda_->forwardKinematics(std::vector<double>(match, match+PANDA_NUM_JOINTS)), "MATCHED GOALEE: ");
                printVec(panda_->forwardKinematics(std::vector<double>(vals, vals+PANDA_NUM_JOINTS)), "TEST STATE EE: ");
                
                
            }
            return dist;
        }


        const std::vector<double> pubGoal_;
    protected:

        /**
         * @brief thread fn to sample a pool of goal states
        */
        virtual bool sampleGoalThread(ompl::base::State *st) const
        {
            std::vector<double> goalPose(goalPose_.begin(), goalPose_.begin() + 6);
            bool good = false;
            // std::cout << "goal Pose extracted";
            unsigned int maxTries = 10000;
            unsigned int tries = 0; 

            do
            {
                for (size_t i = 0; i < 10 && !good; ++i)
                {
                    // printVec(goalPose);
                        // std::cout<<"\n$$$$$$$$$$$$ test0.1\n";

                    std::vector<double> q = panda_->inverseKinematics(goalPose);
                    // std::cout<<"\n$$$$$$$$$$$$ test0.2\n";
                    
                    if (q.empty()) {
                        continue;
                    }
                    while (q.size() > PANDA_NUM_MOVABLE_JOINTS) {
                        q.pop_back();
                    }

                    if (not panda_->inCollision(q))
                    {
                        int numVals = q.size();
                        for(int i = 0; i < 2 * PANDA_NUM_JOINTS - numVals; i++) {
                            q.push_back(0.0);
                        }
                        // copy values into state
                        // std::cout<<"\n$$$$$$$$$$$$ test1\n";
                        double *arr = q.data();
                        // std::cout<<"starting state copy\n";
                        memcpy(st->as<PandaStateSpace::StateType>()->values, arr,
                            2 * PANDA_NUM_JOINTS * sizeof(double));
                        
                        // std::cout<<"\n$$$$$$$$$$$$ test2\n";
                        
                        // GoalLazySamples will check validity
                        good = true;
                       printVec(q, "#### NEW GOAL: ");
                       std::vector<double> subQ(q.begin(), q.begin() + 7);
                       printVec(panda_->forwardKinematics(subQ), "NEW GOAL POSE: ");
                       printVec(goalPose, "### GOAL Pose");
                    }
                    tries++;
                }
            } while (not good and tries < maxTries);
            return good;
        }
    
    private:
        mutable ompl::RNG rng_;
        ompl::base::StateSamplerPtr stateSampler_;
        const std::vector<double> goalPose_;
        std::shared_ptr<RobotInterface> panda_;
        const Eigen::Quaternionf goalQuat_;
};

#endif