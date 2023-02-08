// #include <pandaStatePropogator.hpp>
#include "pandaStatePropogator.hpp"
#include <pandaControlSpace.hpp>
#include <pandaStateSpace.hpp>
#include <kdmpUtils.hpp>

PandaStatePropogator::PandaStatePropogator(const ompl::control::SpaceInformationPtr &si) :
        ompl::control::StatePropagator(si)
{}

void PandaStatePropogator::propagate(const ompl::base::State *start, const ompl::control::Control* control,
                const double duration, ompl::base::State *result) const
{
    double *t = control->as<PandaControlSpace::ControlType>()->values;
    double *x = start->as<PandaStateSpace::StateType>()->values;

    
    std::vector<double> torque(t, t + PANDA_NUM_MOVABLE_JOINTS), state(x, x + 2 * PANDA_NUM_MOVABLE_JOINTS);
    std::vector<double> acc = acc_from_torque(state, torque);

    printVec(torque, "Torque: ");

    printVec(acc, "acceleration: ");
    double dt = si_->getPropagationStepSize();
    double *newState = result->as<PandaStateSpace::StateType>()->values;
    memcpy(newState, &x[0], si_->getStateDimension() * sizeof(double));
    for (int i = 0; i < 1; i++) {
        for (int j = 0; j < PANDA_NUM_MOVABLE_JOINTS; j++) {
            newState[j] += acc[j] * dt * dt;
            newState[j + PANDA_NUM_JOINTS] += acc[j] * dt;
        }
        state = std::vector<double>(newState, newState + si_->getStateDimension());
        acc = acc_from_torque(state, torque);
    }
    printVec(state, "****************************** propogation:");

}