
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

    // cs_.steer(control, source, dstPos);
    // perform the first step of propagation
    statePropagator_->propagate(source, control, stepSize, dest);
    // if we reached the goal, we're done
    double distance;
    if (goal_->isSatisfied(dest, &distance))
        return 1;
    std::cout<<"Distance to goal: \n" << distance;
    // if we found a valid state after one step, we can go on
    if (si_->isValid(dest))
    {
        std::cout<<"is valid\n";
        omplBase::State *temp1 = dest, *temp2 = si_->allocState(), *toDelete = temp2;
        std::cout<<"alocate\n";
        unsigned int r = steps;
        for (unsigned int i = 1 ; i < steps ; ++i)
        {
            statePropagator_->propagate(temp1, control, stepSize, temp2);
            std::cout<<"did propogate \n";
            if (goal_->isSatisfied(dest, &distance))
            {
                std::cout<<"do free\n";
                si_->copyState(dest, temp2);
                std::cout<<"didCopy \n";
                std::cout<<"Free 1\n";
                si_->freeState(toDelete);
                return i + 1;
            }
            std::cout<<"not satisifed\n";
            if (si_->isValid(temp2))
            {
                std::cout<<"doing swap\n";
                std::swap(temp1, temp2);
                std::cout<<"did swap\n";
            }
            else
            {
                // the last valid state is temp1;
                r = i;
                break;
            }
        }
        std::cout<<"Loop done \n";
        // if we finished the for-loop without finding an invalid state, the last valid state is temp1
        // make sure dest contains that information
        if (dest != temp1)
            si_->copyState(dest, temp1);
        std::cout<<"Free 2\n";
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