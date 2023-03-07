#ifndef RIGID_BODY_DYNAMICS_
#define RIGID_BODY_DYNAMICS_

#include <vector>
#include <pandaConstants.hpp>
#include "kdmpUtils.hpp"
#include <franka_model.h>
#include <Eigen/LU>
#include <Eigen/Dense>

static std::vector<double> acc_from_torque(const std::vector<double> &state, const std::vector<double> &torque)
{
   auto q = Eigen::vecToEigenVec(std::vector<double>(state.begin(), state.begin() + PANDA_NUM_JOINTS));
   auto qd = Eigen::vecToEigenVec(std::vector<double>(state.begin() + PANDA_NUM_JOINTS, state.end()));

   // printVec(Eigen::eigenVecTovVec( q),  "$$$$$TOURQUE Q: ");
   // printVec(Eigen::eigenVecTovVec( qd), "$$$$$TOURQUE V: ");

   auto t = Eigen::vecToEigenVec(torque);

   Eigen::Matrix7d M = MassMatrix(q);
   auto C = CoriolisMatrix(q, qd);
   auto g = GravityVector(q);
   auto f = Friction(qd);

   Eigen::Vector7d acc = M.colPivHouseholderQr().solve((t - (C * qd) - g - f));
   
   return Eigen::eigenVecTovVec(acc);
}

static Eigen::MatrixXd get_jacobian_derivative(const Eigen::MatrixXd &J, const std::vector<double> &dqVec)
{
   Eigen::MatrixXd Jdot(J.rows(), J.cols());
   auto dq = Eigen::vecToEigenVec(dqVec);
   for (int i = 0 ; i <  PANDA_NUM_MOVABLE_JOINTS; i++) {
      for (int j = 0; j < PANDA_NUM_MOVABLE_JOINTS; j++) {
         if (i > j) {
            Eigen::VectorXd Ji = J.col(i);
            Eigen::VectorXd Jj = J.col(j);
            auto dJi_dqj = Ji.adjoint() * Jj;
            Jdot.col(i) += dJi_dqj * dq[j];
         }
      }
   }
   return Jdot;
}

static std::vector<double> get_tau(std::vector<double> q, std::vector<double> dq, std::vector<double> ddq)
{
   auto qE = Eigen::vecToEigenVec(q);
   auto qd = Eigen::vecToEigenVec(dq);
   auto qdd = Eigen::vecToEigenVec(ddq);

   auto M = MassMatrix(qE);
   auto C = CoriolisMatrix(qE, qd);
   auto g = GravityVector(qE);
   auto f = Friction(qd);

   auto tau = M * qdd + C * qd + g + f;
   return Eigen::eigenVecTovVec(tau);
}

struct odeData
{
   std::vector<double> tau;
};

static void PandaOde(double t, double* y, double* ydot, void* data)
{
   odeData *panda_data = (odeData *)data;

   std::vector<double> state(y, y + PANDA_NUM_JOINTS * 2);
   
   std::vector<double> dq(y + PANDA_NUM_JOINTS, y + 2 * PANDA_NUM_JOINTS);
   std::vector<double> ddq = acc_from_torque(state, panda_data->tau);
   for (int i = 0; i < PANDA_NUM_MOVABLE_JOINTS; i++)
   {
      ydot[i] = dq[i];
      ydot[i+PANDA_NUM_JOINTS] = ddq[i];
   }
}


#endif