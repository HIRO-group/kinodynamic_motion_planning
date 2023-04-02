#ifndef PANDA_DIRECTED_CONTROL_SAMPLER_
#define PANDA_DIRECTED_CONTROL_SAMPLER_

#include "pandaControlSpace.hpp"
#include "pandaStatePropogator.hpp"

#include <ompl/control/SpaceInformation.h>
#include <ompl/base/Goal.h>
#include <ompl/control/SimpleDirectedControlSampler.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

class PandaDirectedControlSampler : public ompl::control::DirectedControlSampler
{
    public:
        PandaDirectedControlSampler(const ompl::control::SpaceInformation *si,
            const ompl::base::GoalPtr &goal, std::shared_ptr<RobotInterface> rbt, int k = 10)
            : DirectedControlSampler(si), cs_(si->getControlSpace().get()),
            goal_(goal), numControlSamples_(k), panda_(rbt)
        {
            statePropagator_ = dynamic_cast<const PandaStatePropogator *>(si->getStatePropagator().get());
        }

        virtual unsigned int sampleTo(ompl::control::Control *control, const ompl::base::State *source, ompl::base::State *dest);

        virtual unsigned int sampleTo(ompl::control::Control *control, const ompl::control::Control * /* previous */,
            const ompl::base::State *source, ompl::base::State *dest)
        {
            return sampleTo(control, source, dest);
        }
    protected:
        unsigned int getBestControl(ompl::control::Control *control, const ompl::base::State *source,
                                                                         ompl::base::State *dest, const ompl::control::Control *previous);
        unsigned int propagateWhileValid(ompl::control::Control *control, const ompl::base::State *source, ompl::base::State *dest, int steps);
        double distance(const ompl::base::State *state1, const ompl::base::State *state2) const;
        ompl::control::RealVectorControlUniformSampler                     cs_;
        ompl::RNG                                rng_;
        const ompl::base::GoalPtr                goal_;
        const PandaStatePropogator *statePropagator_;
        bool                                     propogateMax_;
        std::shared_ptr<RobotInterface> panda_;
        const LSODA lsoda_;
        int numControlSamples_;
};


#endif