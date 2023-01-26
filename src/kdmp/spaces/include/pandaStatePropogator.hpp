#ifndef PANDA_STATEPROPAGATOR_
#define PANDA_STATEPROPAGATOR_

#include <ompl/control/StatePropagator.h>
#include <kdmp/models/include/rigidBodyDynamics.hpp>



class PandaStatePropogator : public ompl::control::StatePropagator
{
    public:
        PandaStatePropogator(const ompl::control::SpaceInformationPtr &si);
        ~PandaStatePropogator();

        virtual void propagate(const ompl::base::State *start, const ompl::control::Control* control,
                const double duration, ompl::base::State *result) const;

};

#endif