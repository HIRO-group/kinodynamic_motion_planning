// #include <pandaStatePropogator.hpp>
#include "pandaStatePropogator.hpp"
#include <pandaControlSpace.hpp>
#include <pandaStateSpace.hpp>

PandaStatePropogator::PandaStatePropogator(const ompl::control::SpaceInformationPtr &si) :
        ompl::control::StatePropagator(si)
{}

void PandaStatePropogator::propagate(const ompl::base::State *start, const ompl::control::Control* control,
                const double duration, ompl::base::State *result) const
{
    double *t = control->as<PandaControlSpace::ControlType>()->values;
    double *x = start->as<PandaStateSpace::StateType>()->values;

    std::vector<double> torque(t, t + PANDA_NUM_JOINTS), state(x, x + 2 * PANDA_NUM_JOINTS);

    std::vector<double> acc = acc_from_torque(state, torque);

    int steps = duration / si_->getPropagationStepSize();
    double dt = si_->getPropagationStepSize();
    double *newState = result->as<PandaStateSpace::StateType>()->values;
    for (int i = 0; i < steps; i++) {
        for (int j = 0; j < PANDA_NUM_JOINTS; j++) {
            newState[i] = acc[i] * dt * dt;
            newState[i + PANDA_NUM_JOINTS] = acc[i] * dt;
        }
        state = std::vector<double>(newState, newState + si_->getStateDimension());
        acc = acc_from_torque(state, torque);
    }
}