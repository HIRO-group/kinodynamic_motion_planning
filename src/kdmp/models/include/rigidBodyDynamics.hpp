#ifndef RIGID_BODY_DYNAMICS_
#define RIGID_BODY_DYNAMICS_

#include <vector>
#include <kdmp/spaces/include/pandaConstants.hpp>
#include <kdmp/models/include/franka_model.h>

std::vector<double> acc_from_torque(const std::vector<double> &state, const std::vector<double> &torque)
{
   auto q = Eigen::Vector7d(std::vector<double>(&state[0], &state[PANDA_NUM_JOINTS - 1]));
   auto qd = Eigen::Vector7d(std::vector<double>(&state[PANDA_NUM_JOINTS], &state[2 * PANDA_NUM_JOINTS - 1]));
   auto t = Eigen::Vector7d(torque);

   auto M_inv = MassMatrix(q).inverse();
   auto C = CoriolisMatrix(q, qd);
   auto g = GravityVector(q);
   auto f = Friction(qd);

   Eigen::Vector7d acc = M_inv * (t - (C * qd) - g - f);
   
   return std::vector<double>(acc.data(), acc.data()+acc.size());
}

#endif