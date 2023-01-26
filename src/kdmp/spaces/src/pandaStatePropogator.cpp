#include <kdmp/spaces/include/pandaStatePropogator.hpp>
#include <kdmp/spaces/include/pandaControlSpace.hpp>
#include <kdmp/spaces/include/pandaStateSpace.hpp>


PandaStatePropogator::PandaStatePropogator(const ompl::control::SpaceInformationPtr &si) :
        ompl::control::StatePropagator(si)
{}

void PandaStatePropogator::propagate(const ompl::base::State *start, const ompl::control::Control* control,
                const double duration, ompl::base::State *result) const
{
    double *t = control->as<PandaControlSpace::ControlType>()->values;
    double *x = start->as<PandaStateSpace::StateType>()->values;

    std::vector<double> torque, state;
    std::copy(t, t + PANDA_NUM_JOINTS, torque);
    std::copy(x, x + si_->getStateDimension(), state);

    std::vector<double> acc = acc_from_torque(state, torque);

    int steps = duration / si_->getPropagationStepSize();
    double dt = si_->getPropagationStepSize();
    double *newState = result->as<PandaStateSpace::StateType>()->values;
    for (int i = 0; i < steps; i++) {
        for (int j = 0; j < PANDA_NUM_JOINTS; j++) {
            newState[i] = acc[i] * dt * dt;
            newState[i + PANDA_NUM_JOINTS] = acc[i] * dt;
        }
        std::copy(newState, newState + si_->getStateDimension(), state);
        acc = acc_from_torque(state, torque);
    }
}