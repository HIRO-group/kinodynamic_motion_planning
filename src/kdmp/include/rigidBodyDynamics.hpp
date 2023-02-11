#ifndef RIGID_BODY_DYNAMICS_
#define RIGID_BODY_DYNAMICS_

#include <vector>
#include <pandaConstants.hpp>
#include <franka_model.h>
#include <Eigen/LU>

static std::vector<double> acc_from_torque(const std::vector<double> &state, const std::vector<double> &torque)
{
   auto q = Eigen::vecToEigenVec(std::vector<double>(&state[0], &state[PANDA_NUM_MOVABLE_JOINTS-1]));
   auto qd = Eigen::vecToEigenVec(std::vector<double>(&state[PANDA_NUM_MOVABLE_JOINTS], &state[2 * PANDA_NUM_MOVABLE_JOINTS -1]));
   auto t = Eigen::vecToEigenVec(torque);

   auto M_inv = MassMatrix(q).inverse();
   auto C = CoriolisMatrix(q, qd);
   auto g = GravityVector(q);
   auto f = Friction(qd);

   Eigen::Vector7d acc = M_inv * (t - (C * qd) - g - f);
   
   return std::vector<double>(acc.data(), acc.data()+acc.size());
}

static Eigen::MatrixXd get_jacobian_derivative(const Eigen::MatrixXd &J, const std::vector<double> &dqVec)
{
   Eigen::MatrixXd Jdot(J.rows(), J.cols());
   std::cout<<"init jdot \n";
   auto dq = Eigen::vecToEigenVec(dqVec);
   std::cout<<"got dq \n";
   for (int i = 0 ; i <  PANDA_NUM_MOVABLE_JOINTS; i++) {
      for (int j = 0; j < PANDA_NUM_MOVABLE_JOINTS; j++) {
         if (i > j) {
            Eigen::VectorXd Ji = J.col(i);
            Eigen::VectorXd Jj = J.col(j);
            std::cout<<"got js \n";
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

   auto tau = M * qdd + C *qd + g + f;
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
   ydot[PANDA_NUM_MOVABLE_JOINTS] = 0.0;
   ydot[2 * PANDA_NUM_JOINTS -1] = 0.0;
}


#endif