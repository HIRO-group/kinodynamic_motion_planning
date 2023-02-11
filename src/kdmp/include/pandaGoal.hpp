#ifndef PANDA_GOAL_
#define PANDA_GOAL_

#include <ompl/base/goals/GoalLazySamples.h>
#include <RobotInterface.hpp>
#include <pandaStateSpace.hpp>
#include "kdmpUtils.hpp"


#include <boost/math/constants/constants.hpp>
#include <iostream>

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
            true), stateSampler_(si->allocStateSampler()), goalPose_(goalState), panda_(rbt), pubGoal_(goalState)
        {
            threshold_ = 0.001;
            goalQuat_ = Eigen::AngleAxisf(goalState[3], Eigen::Vector3f::UnitX())
                * Eigen::AngleAxisf(goalState[4], Eigen::Vector3f::UnitY())
                * Eigen::AngleAxisf(goalState[5], Eigen::Vector3f::UnitZ());
        }

        /**
         * @brief find the euler distance from st to the goal state
         * 
         * @param[in] st - state whose distance from goal is being checked
        */
        virtual double distanceGoal(const ompl::base::State *st) const
        {
            std::cout<<"in distance goal \n";
            const double *state = st->as<PandaStateSpace::StateType>()->values;
            std::vector<double> confVec(state, state + PANDA_NUM_MOVABLE_JOINTS);
            std::vector<double> velVec(state + PANDA_NUM_JOINTS + 1, state + 2 * PANDA_NUM_JOINTS - 1);
            std::cout<<"setup vel and conf vecs\n";
            std::vector<double> poseWorld = panda_->forwardKinematics(confVec);
            std::cout<<"did fk\n";
            std::vector<double> velWorld = panda_->jointVelToEeVel(velVec, confVec);
            std::cout<<"did qd to ee vel \n";
            std::vector<double> stateWorld;
            for (int i = 0; i < poseWorld.size(); i++) {
                stateWorld.push_back(poseWorld[i]);
            }
            for (int i = 0; i < velWorld.size(); i++) {
                stateWorld.push_back(velWorld[i]);
            }
            
            printVec(confVec, "joint poses: ");
            printVec(velVec, "joint vel: ");
            printVec(stateWorld, "goal test: ");
            printVec(goalPose_, "goal: ");
            Eigen::Quaternionf stateQuat = Eigen::AngleAxisf(stateWorld[3], Eigen::Vector3f::UnitX())
                * Eigen::AngleAxisf(stateWorld[4], Eigen::Vector3f::UnitY())
                * Eigen::AngleAxisf(stateWorld[5], Eigen::Vector3f::UnitZ());
            
            Eigen::Quaternionf diffQuat;
            diffQuat = stateQuat * goalQuat_.inverse();

            std::vector<double> diffEuler = Eigen::eigenVecTovVec(diffQuat.toRotationMatrix().eulerAngles(0, 1, 2));
            
            std::cerr<<"setup stateWorld\n";
            double dist = 0;
            for (int i = 0; i < goalPose_.size(); i++) {
                dist += std::pow(goalPose_[i] - stateWorld[i], 2);
            }
            for (int i = 0; i < diffEuler.size(); i++) {
                dist += std::pow(diffEuler[i], 2); 
            }
            std::cerr<< " found dist: " << std::sqrt(dist) << std::endl;

            return std::sqrt(dist);
        }


        const std::vector<double> pubGoal_;
    protected:

        /**
         * @brief thread fn to sample a pool of goal states
        */
        virtual bool sampleGoalThread(ompl::base::State *st) const
        {
            std::cout << "in goal thread sample\n";
            std::vector<double> goalPose(goalPose_.begin(), goalPose_.begin() + 6);
            bool good = false;
            std::cout << "goal Pose extracted";
            unsigned int maxTries = 1000;
            unsigned int tries = 0; 

            do
            {
                for (size_t i = 0; i < 10 && !good; ++i)
                {
                    printVec(goalPose);
                    std::vector<double> q = panda_->inverseKinematics(goalPose);
                    
                    if (q.empty()) {
                        continue;
                    }
                    std::cout<< "ik state size: "<< q.size() <<std::endl;
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
                        memcpy(st->as<PandaStateSpace::StateType>()->values, &q[0],
                            q.size() * sizeof(double));
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
        std::vector<double> goalPose_;
        std::shared_ptr<RobotInterface> panda_;
        Eigen::Quaternionf goalQuat_;
};

#endif