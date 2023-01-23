#include <kdmp/spaces/include/pandaControlSpace.hpp>
#include <kdmp/spaces/include/pandaStateSpace.hpp>

namespace omplBase = ompl::base;
namespace omplControl = ompl::control;

PandaControlSpace::PandaControlSpace(int numDims, PandaControlType controlType) :
        mNumDims(numDims), mControlType(controlType)
{
    auto limits = mControlType == VELOCITY_CTL ? PANDA_VEL_LIMS : PANDA_TORQUE_LIMS;
    for (int i = 0; i < PANDA_NUM_JOINTS; i++) {
        bounds_.setLow(i, limits[i][0]);
        bounds_.setHigh(i, limits[i][1]);
    }
}

void PandaControlSampler::sample(oc::Control *control)
{
    const ob::RealVectorBounds &bounds = space_->as<oc::RealVectorControlSpace>()->getBounds();
    auto *rcontrol =
        control->as<oc::RealVectorControlSpace::ControlType>();
    for (int i = 0; i < mNumDims; i++) {
        double qdi = rng_.uniformReal(bounds.low[0], bounds.high[0]);
        rcontrol->values[i] = qdi;
    }
}

void PandaControlSampler::sample(ompl::control::Control *control, const ompl::base::State *state)
{
    std::vector<double> x;
    for (int i = 0; i < PANDA_NUM_JOINTS; i++) {
        x.push_back(rng_.uniformReal(PANDA_JOINT_LIMS[i][0], PANDA_JOINT_LIMS[i][1]));
    }
    steer(control, state, x);
}

void PandaControlSampler::steer(ompl::control::Control *control, const ompl::base::State *state, std::vector<double> x)
{
    const double* q = state->as<KoulesStateSpace::StateType>()->values;

    for (int i = 0; i < PANDA_JOINT_LIMS; i++) {
        double diff = x[i] - q[i];
        double qdi = std::abs(diff) < std::numeric_limits<float>::epsilon() ? 0 : rng_.uniformReal(0, bounds.high[0]);
        rcontrol->values[i] = diff < 0 ? -1 * qdi : qdi;
    }
}