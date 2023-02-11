#ifndef PANDA_STATEPROPAGATOR_
#define PANDA_STATEPROPAGATOR_

#include <ompl/control/StatePropagator.h>
#include <ompl/control/ODESolver.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>

#include "pandaStateSpace.hpp"
#include "pandaControlSpace.hpp"
#include "rigidBodyDynamics.hpp"
#include "RobotInterface.hpp"
#include "LSODA.h"


class PandaStatePropogator : public ompl::control::StatePropagator
{
    public:
        PandaStatePropogator(const ompl::control::SpaceInformationPtr &si, std::shared_ptr<RobotInterface> &robot_interface);

        virtual void propagate_temp(const ompl::base::State *start, const ompl::control::Control* control,
                const double duration, ompl::base::State *result) const;
        virtual void propagate(const ompl::base::State *start, const ompl::control::Control* control,
                const double duration, ompl::base::State *result) const;
    private:
        LSODA lsoda;
        std::shared_ptr<RobotInterface> panda_;
};


#endif