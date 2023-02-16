
#include <pandaDirectedControlSampler.hpp>
#include <pandaStateSpace.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include "pandaStatePropogator.hpp"


namespace omplBase = ompl::base;
namespace omplControl = ompl::control;

 unsigned int PandaDirectedControlSampler::sampleTo(omplControl::Control *control, const omplBase::State *source, omplBase::State *dest)
 {
    return getBestControl(control, source, dest, nullptr);

 }

 unsigned int PandaDirectedControlSampler::getBestControl(ompl::control::Control *control, const ompl::base::State *source,
                                                        ompl::base::State *dest, const ompl::control::Control *previous)
{
    // Sample the first control
    if (previous != nullptr)
        cs_.sampleNext(control, previous, source);
    else
        cs_.sample(control);
    const unsigned int minDuration = si_->getMinControlDuration();
    const unsigned int maxDuration = si_->getMaxControlDuration();

    unsigned int steps = cs_.sampleStepCount(minDuration, maxDuration);
    // Propagate the first control, and find how far it is from the target state
    ompl::base::State *bestState = si_->allocState();
    lsoda_.prev_time_ = 0.0;
    steps = si_->propagateWhileValid(source, control, steps, bestState);

    if (numControlSamples_ > 1)
    {
        ompl::control::Control *tempControl = si_->allocControl();
        ompl::base::State *tempState = si_->allocState();
        double bestDistance = distance(bestState, dest);

        // Sample k-1 more controls, and save the control that gets closest to target
        for (unsigned int i = 1; i < numControlSamples_; ++i)
        {
            unsigned int sampleSteps = cs_.sampleStepCount(minDuration, maxDuration);
            if (previous != nullptr)
                cs_.sampleNext(tempControl, previous, source);
            else
                cs_.sample(tempControl);
            lsoda_.prev_time_ = i/PANDA_CTL_RATE;
            sampleSteps = si_->propagateWhileValid(source, tempControl, sampleSteps, tempState);
            double tempDistance = distance(tempState, dest);
            if (tempDistance < bestDistance)
            {
                si_->copyState(bestState, tempState);
                si_->copyControl(control, tempControl);
                bestDistance = tempDistance;
                steps = sampleSteps;
            }
        }

        si_->freeState(tempState);
        si_->freeControl(tempControl);
    }

    si_->copyState(dest, bestState);
    si_->freeState(bestState);

    return steps;
}

unsigned int PandaDirectedControlSampler::propagateWhileValid(ompl::control::Control *control, ompl::base::State *source, ompl::base::State *dest, int steps)
{
    const double* dstPos = dest->as<PandaStateSpace::StateType>()->values;
    double stepSize = si_->getPropagationStepSize();

    // perform the first step of propagation
    statePropagator_->propagate(source, control, stepSize, dest, lsoda_);
    // if we reached the goal, we're done
    if (goal_->isSatisfied(dest))
        return 1;
    // if we found a valid state after one step, we can go on
    if (si_->isValid(dest))
    {
        ompl::base::State *temp1 = dest, *temp2 = si_->allocState(), *toDelete = temp2;
        unsigned int r = steps;
        for (unsigned int i = 1 ; i < steps ; ++i)
        {
            lsoda_.prev_time_ += 1/PANDA_CTL_RATE;
            statePropagator_->propagate(temp1, control, stepSize, temp2, lsoda_);
            if (goal_->isSatisfied(dest))
            {
                si_->copyState(dest, temp2);
                si_->freeState(toDelete);
                return i + 1;
            }
            if (si_->isValid(temp2))
                std::swap(temp1, temp2);
            else
            {
                // the last valid state is temp1;
                r = i;
                break;
            }
        }
        // if we finished the for-loop without finding an invalid state, the last valid state is temp1
        // make sure dest contains that information
        if (dest != temp1)
            si_->copyState(dest, temp1);
        si_->freeState(toDelete);
        return r;
    }
    // if the first propagation step produced an invalid step, return 0 steps
    // the last valid state is the starting one (assumed to be valid)
    else
    {
        if (dest != source)
            si_->copyState(dest, source);
        return 0;
    }
}

double PandaDirectedControlSampler::distance(const ompl::base::State *state1, const ompl::base::State *state2) const
{
            double *state = state1->as<PandaStateSpace::StateType>()->values;
            std::vector<double> confVec(state, state + PANDA_NUM_MOVABLE_JOINTS);
            std::vector<double> velVec(state + PANDA_NUM_JOINTS + 1, state + 2 * PANDA_NUM_JOINTS - 1);
            std::vector<double> poseWorld = panda_->forwardKinematics(confVec);
            std::vector<double> velWorld = panda_->jointVelToEeVel(velVec, confVec);
            std::vector<double> stateWorld;
            for (int i = 0; i < poseWorld.size(); i++) {
                stateWorld.push_back(poseWorld[i]);
            }
            for (int i = 0; i < velWorld.size(); i++) {
                stateWorld.push_back(velWorld[i]);
            }
            
            state = state2->as<PandaStateSpace::StateType>()->values;
            std::vector<double> confVec2(state, state + PANDA_NUM_MOVABLE_JOINTS);
            std::vector<double> velVec2(state + PANDA_NUM_JOINTS + 1, state + 2 * PANDA_NUM_JOINTS - 1);
            std::vector<double> poseWorld2 = panda_->forwardKinematics(confVec2);
            std::vector<double> velWorld2 = panda_->jointVelToEeVel(velVec2, confVec2);
            std::vector<double> goalPose_;
            for (int i = 0; i < poseWorld2.size(); i++) {
                goalPose_.push_back(poseWorld2[i]);
            }
            for (int i = 0; i < velWorld2.size(); i++) {
                goalPose_.push_back(velWorld2[i]);
            }
            
            Eigen::Quaternionf stateQuat = Eigen::AngleAxisf(stateWorld[3], Eigen::Vector3f::UnitX())
                * Eigen::AngleAxisf(stateWorld[4], Eigen::Vector3f::UnitY())
                * Eigen::AngleAxisf(stateWorld[5], Eigen::Vector3f::UnitZ());
            Eigen::Quaternionf goalQuat_ = Eigen::AngleAxisf(goalPose_[3], Eigen::Vector3f::UnitX())
                * Eigen::AngleAxisf(goalPose_[4], Eigen::Vector3f::UnitY())
                * Eigen::AngleAxisf(goalPose_[5], Eigen::Vector3f::UnitZ());
            
            Eigen::Quaternionf diffQuat;
            diffQuat = stateQuat * goalQuat_.inverse();

            std::vector<double> diffEuler = Eigen::eigenVecTovVec(diffQuat.toRotationMatrix().eulerAngles(0, 1, 2));
            
            std::cerr<<"setup stateWorld\n";
            double dist = 0;
            for (int i = 0; i < 3; i++) {
                dist += std::pow(goalPose_[i] - stateWorld[i], 2);
            }
            for (int i = 0; i < diffEuler.size(); i++) {
                dist += std::pow(diffEuler[i], 2); 
            }
            for(int i = 0; i < velWorld.size(); i ++) {
                dist += std::pow(velWorld[i], 2);
            }
            std::cerr<< " found dist: " << std::sqrt(dist) << std::endl;

            return std::sqrt(dist);
}