#ifndef PANDA_DIRECTED_CONTROL_SAMPLER_
#define PANDA_DIRECTED_CONTROL_SAMPLER_

#include "kdmp/spaces/include/pandaControlSpace.hpp"
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/Goal.h>

class PandaDirectedControlSampler : public ompl::control::DirectedControlSampler
{
    public:
        PandaDirectedControlSampler(const ompl::control::SpaceInformation *si,
            const ompl::base::GoalPtr &goal, bool propagateMax)
            : DirectedControlSampler(si), cs_(si->getControlSpace().get()),
            goal_(goal), statePropagator_(si->getStatePropagator()),
            propagateMax_(propagateMax)
        {
        }

        virtual unsigned int sampleTo(ompl::control::Control *control, const ompl::base::State *source, ompl::base::State *dest);

        virtual unsigned int sampleTo(ompl::control::Control *control, const ompl::control::Control * /* previous */,
            const ompl::base::State *source, ompl::base::State *dest)
        {
            return sampleTo(control, source, dest);
        }
    protected:
        PandaControlSampler                      cs_;
        ompl::RNG                                rng_;
        const ompl::base::GoalPtr                goal_;
        const ompl::control::StatePropagatorPtr  statePropagator_;
        bool                                     propogateMax_;
};


#endif