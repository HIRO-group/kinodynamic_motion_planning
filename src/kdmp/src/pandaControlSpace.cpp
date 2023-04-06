#include <pandaControlSpace.hpp>
#include <pandaStateSpace.hpp>
#include <rigidBodyDynamics.hpp>
#include <kdmpUtils.hpp>

namespace omplBase = ompl::base;
namespace omplControl = ompl::control;

PandaControlSpace::PandaControlSpace(PandaControlType controlType, int numDims) :
        mNumDims(numDims), mControlType(controlType), ompl::control::RealVectorControlSpace(
        std::make_shared<PandaStateSpace>(), numDims)
{
    // auto limits = PANDA_TORQUE_LIMS;
    // for (int i = 0; i < numDims; i++) {
    //     bounds_.setLow(i, limits[i][0]);
    //     bounds_.setHigh(i, limits[i][1]);
    // }
    bounds_.setLow(-0.8);
    bounds_.setHigh(0.8);
}

void PandaControlSampler::sample(omplControl::Control *control)
{
    const omplBase::RealVectorBounds &bounds = space_->as<omplControl::RealVectorControlSpace>()->getBounds();
    int numDims = space_->getDimension();
    auto *rcontrol =
        control->as<omplControl::RealVectorControlSpace::ControlType>();
    for (int i = 0; i < numDims; i++) {
        double mag = rng_.uniformReal(0.1, 0.8);
        double dir = rng_.uniformReal(-1.0, 1.0);

        rcontrol->values[i] = mag*dir;
    }
}