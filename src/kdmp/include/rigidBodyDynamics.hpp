#ifndef RIGID_BODY_DYNAMICS_
#define RIGID_BODY_DYNAMICS_

#include <vector>
#include <pandaConstants.hpp>
#include <franka_model.h>
#include <Eigen/LU>

static std::vector<double> acc_from_torque(const std::vector<double> &state, const std::vector<double> &torque)
{
   auto q = Eigen::vecToEigenVec(std::vector<double>(&state[0], &state[PANDA_NUM_JOINTS - 2]));
   auto qd = Eigen::vecToEigenVec(std::vector<double>(&state[PANDA_NUM_JOINTS], &state[2 * PANDA_NUM_JOINTS - 2]));
   auto t = Eigen::vecToEigenVec(torque);

   auto M_inv = MassMatrix(q).inverse();
   auto C = CoriolisMatrix(q, qd);
   auto g = GravityVector(q);
   auto f = Friction(qd);

   Eigen::Vector7d acc = M_inv * (t - (C * qd) - g - f);
   
   return std::vector<double>(acc.data(), acc.data()+acc.size());
}

#endif