#include "pandaControlTorque.hpp"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include <Eigen/Core>

void PandaControlTorque::ODE(const omplControl::ODESolver::StateType& x, const omplControl::Control* c, omplControl::ODESolver::StateType& qdd)
{
    const double *u = c->as<omplControl::RealVectorControlSpace::ControlType>()->values;
    std::vector<double> q(&x[0], &x[PANDA_NUM_JOINTS]);
    int velStart = PANDA_NUM_JOINTS + 1;
    int velEnd = velStart + PANDA_NUM_JOINTS;
    std::vector<double> qd(&x[velStart], &x[velEnd]);
    const Eigen::Vector7d qVec(q.data());
    const Eigen::Vector7d qdVec(qd.data());

    Eigen::Matrix7d M = MassMatrix(qVec);
    Eigen::Matrix7d C = CoriolisMatrix(qVec, qdVec);
    auto G = GravityVector(qVec);
    auto F = Friction(qdVec);

    
}