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
    std::cerr<<"############################### sample"<<std::endl;
    const omplBase::RealVectorBounds &bounds = space_->as<omplControl::RealVectorControlSpace>()->getBounds();
    int numDims = space_->getDimension();
    auto *rcontrol =
        control->as<omplControl::RealVectorControlSpace::ControlType>();
    for (int i = 0; i < numDims; i++) {
        double t = rng_.uniformReal(bounds.low[i], bounds.high[i]);
        rcontrol->values[i] = t;
    }
}

void PandaControlSampler::sample(ompl::control::Control *control, const ompl::base::State *state)
{
    std::cerr<<"sampling ctl"<<std::endl;
    std::vector<double> x;
    for (int i = 0; i < 2 * PANDA_NUM_MOVABLE_JOINTS; i++) {
        if (i < PANDA_NUM_MOVABLE_JOINTS) {
            x.push_back(rng_.uniformReal(PANDA_JOINT_LIMS[i][0], PANDA_JOINT_LIMS[i][1]));
        } else {
            x.push_back(rng_.uniformReal(PANDA_VEL_LIMS[i - PANDA_NUM_MOVABLE_JOINTS][0], PANDA_VEL_LIMS[i - PANDA_NUM_MOVABLE_JOINTS][1]));
        }
    }
    steer(control, state, x);
}

void PandaControlSampler::steer(ompl::control::Control *control, const ompl::base::State *state, std::vector<double> x)
{
    // std::cerr<<"In steer"<<std::endl;
    // const omplBase::RealVectorBounds &bounds = space_->as<omplControl::RealVectorControlSpace>()->getBounds();
    // const double* q = state->as<PandaStateSpace::StateType>()->values;
    // auto *rcontrol =
    //     control->as<omplControl::RealVectorControlSpace::ControlType>();
    // std::vector<double> diff(2*PANDA_NUM_MOVABLE_JOINTS, 0.0), t, a, neg_a, neg_t;
    // for (int i = 0; i < PANDA_NUM_MOVABLE_JOINTS; i++) {
    //     diff[i] = (x[i] - q[i]);
    //     if (std::abs(diff[i]) < std::numeric_limits<float>::epsilon()) {
    //         rcontrol->values[i] = 0;
    //     } else if (diff[i] < 0) {
    //        rcontrol->values[i] =  rng_.uniformReal(bounds.low[i], 0);
    //     } else {
    //          rcontrol->values[i] =  rng_.uniformReal(0, bounds.high[i]);
    //     }
    // }
    sample(control);
}