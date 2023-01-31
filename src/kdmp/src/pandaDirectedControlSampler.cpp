
#include <pandaDirectedControlSampler.hpp>
#include <pandaStateSpace.hpp>
#include <ompl/base/spaces/RealVectorStateSpace.h>

namespace omplBase = ompl::base;
namespace omplControl = ompl::control;

 unsigned int PandaDirectedControlSampler::sampleTo(omplControl::Control *control, const omplBase::State *source, omplBase::State *dest)
 {
    double *temp = dest->as<PandaStateSpace::StateType>()->values;
    const std::vector<double> dstPos(temp, temp + si_->getStateDimension());
    double stepSize = si_->getPropagationStepSize();
    unsigned int steps = 
        cs_.sampleStepCount(si_->getMinControlDuration(), si_->getMaxControlDuration());

    cs_.steer(control, source, dstPos);
    // perform the first step of propagation
    statePropagator_->propagate(source, control, stepSize, dest);
    // if we reached the goal, we're done
    if (goal_->isSatisfied(dest))
        return 1;
    // if we found a valid state after one step, we can go on
    if (si_->isValid(dest))
    {
        omplBase::State *temp1 = dest, *temp2 = si_->allocState(), *toDelete = temp2;
        unsigned int r = steps;
        for (unsigned int i = 1 ; i < steps ; ++i)
        {
            statePropagator_->propagate(temp1, control, stepSize, temp2);
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