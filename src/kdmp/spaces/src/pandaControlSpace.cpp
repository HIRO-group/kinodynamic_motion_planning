#include <kdmp/spaces/include/pandaControlSpace.hpp>
#include <kdmp/spaces/include/pandaStateSpace.hpp>
#include <kdmp/models/include/rigidBodyDynamics.hpp>

namespace omplBase = ompl::base;
namespace omplControl = ompl::control;

PandaControlSpace::PandaControlSpace(PandaControlType controlType, int numDims) :
        mNumDims(numDims), mControlType(controlType)
{
    auto limits = mControlType == VELOCITY_CTL ? PANDA_VEL_LIMS : PANDA_TORQUE_LIMS;
    for (int i = 0; i < PANDA_NUM_JOINTS; i++) {
        bounds_.setLow(i, limits[i][0]);
        bounds_.setHigh(i, limits[i][1]);
    }
}

void PandaControlSampler::sample(omplControl::Control *control)
{
    const omplBase::RealVectorBounds &bounds = space_->as<omplControl::RealVectorControlSpace>()->getBounds();
    int numDims = space_->getDimension();
    auto *rcontrol =
        control->as<omplControl::RealVectorControlSpace::ControlType>();
    for (int i = 0; i < numDims; i++) {
        double t = rng_.uniformReal(bounds.low[0], bounds.high[0]);
        rcontrol->values[i] = t;
    }
}

void PandaControlSampler::sample(ompl::control::Control *control, const ompl::base::State *state)
{
    std::vector<double> x;
    for (int i = 0; i < 2 * PANDA_NUM_JOINTS; i++) {
        if (i < PANDA_NUM_JOINTS) {
            x.push_back(rng_.uniformReal(PANDA_JOINT_LIMS[i][0], PANDA_JOINT_LIMS[i][1]));
        } else {
            x.push_back(rng_.uniformReal(PANDA_VEL_LIMS[i - PANDA_NUM_JOINTS][0], PANDA_VEL_LIMS[i - PANDA_NUM_JOINTS][1]));
        }
    }
    steer(control, state, x);
}

void PandaControlSampler::steer(ompl::control::Control *control, const ompl::base::State *state, std::vector<double> x)
{
    const omplBase::RealVectorBounds &bounds = space_->as<omplControl::RealVectorControlSpace>()->getBounds();
    const double* q = state->as<PandaStateSpace::StateType>()->values;
    auto *rcontrol =
        control->as<omplControl::RealVectorControlSpace::ControlType>();
    std::vector<double> diff, t, a, neg_a, neg_t;
    for (int i = 0; i < 2 * PANDA_NUM_JOINTS; i++) {
        diff.push_back(x[i] - q[i]);
        t.push_back(std::abs(diff[i]) < std::numeric_limits<float>::epsilon() ? 0 : rng_.uniformReal(0, bounds.high[0]));
        neg_t.push_back(-1*t[i]);
    }
    std::vector<double> q_vec;
    std::copy(q, q + space_->getDimension(), q_vec);
    a = acc_from_torque(q_vec, t);
    neg_a = acc_from_torque(q_vec, neg_t);

    for (int i = 0; i < PANDA_NUM_JOINTS; i++) {
        if (diff[i] * diff[i + PANDA_NUM_JOINTS] <= 0 and diff[i + PANDA_NUM_JOINTS] <= 0) {
            // velocity is 0 or in the wrong direction
            rcontrol->values[i] = neg_a[i] > a[i] ? neg_t[i] : t[i];
        } else {
            rcontrol->values[i] = neg_a[i] < a[i] ? neg_t[i] : t[i];
        }
    }
}