// #include <pandaStatePropogator.hpp>
#include "pandaStatePropogator.hpp"
#include <pandaControlSpace.hpp>
#include <pandaStateSpace.hpp>
#include <kdmpUtils.hpp>

PandaStatePropogator::PandaStatePropogator(const ompl::control::SpaceInformationPtr &si, std::shared_ptr<RobotInterface> &robot_interface) :
        ompl::control::StatePropagator(si), panda_(robot_interface)
{}

void PandaStatePropogator::propagate_temp(const ompl::base::State *start, const ompl::control::Control* control,
                const double duration, ompl::base::State *result) const
{
    double *t = control->as<PandaControlSpace::ControlType>()->values;
    double *x = start->as<PandaStateSpace::StateType>()->values;

    
    std::vector<double> torque(t, t + PANDA_NUM_MOVABLE_JOINTS), state(x, x + 2 * PANDA_NUM_JOINTS);
    state.erase(state.begin() + PANDA_NUM_MOVABLE_JOINTS); state.erase(state.end() - 1);
    std::vector<double> acc = acc_from_torque(state, torque);

    printVec(torque, "Torque: ");

    printVec(acc, "acceleration: ");
    double dt = si_->getPropagationStepSize();
    int steps = duration / dt;
    double *newState = result->as<PandaStateSpace::StateType>()->values;
    memcpy(newState, &x[0], si_->getStateDimension() * sizeof(double));
    for (int i = 0; i < steps; i++) {
        for (int j = 0; j < PANDA_NUM_MOVABLE_JOINTS; j++) {
            newState[j] += acc[j] * dt * dt;
            newState[j + PANDA_NUM_JOINTS] += acc[j] * dt;
        }
        state = std::vector<double>(newState, newState + si_->getStateDimension());
        acc = acc_from_torque(state, torque);
    }
    printVec(state, "****************************** propogation:");
}

void PandaStatePropogator::propagate(const ompl::base::State *start, const ompl::control::Control* control,
                const double duration, ompl::base::State *result) const
{
    double *t = control->as<PandaControlSpace::ControlType>()->values;
    double *x = start->as<PandaStateSpace::StateType>()->values;
    double *q_next = result->as<PandaStateSpace::StateType>()->values;
    std::cout << "in propogate!!!"<<std::endl;
    std::vector<double> eeVel(t, t + 6), state(x, x + 2 * PANDA_NUM_JOINTS);
    std::vector<double> q(x, x+PANDA_NUM_JOINTS);
    std::vector<double> dq = panda_->eeVelToJointVel(eeVel, q);
    std::vector<double> ee_acc(PANDA_NUM_MOVABLE_JOINTS, 0.0);
    std::cout << "eevel to joint vel worked"<<std::endl;
    std::vector<double> ddq = panda_->ddqFromEEAcc(ee_acc, dq, q);
    auto boundsA = PANDA_ACC_LIMS;
    for (int i = 0; i < PANDA_NUM_MOVABLE_JOINTS; i++) {
        if (boundsA[i][0] > ddq[i] or boundsA[i][1] < ddq[i]) {
            double inf = std::numeric_limits<double>::infinity();
            for (int j = 0; j < 2 * PANDA_NUM_JOINTS; j++) {
                q_next[j] = inf;
            }
            return;
        }
    }

    std::vector<double> tau = get_tau(q, dq, ddq);
    auto boundsT = PANDA_TORQUE_LIMS;
    for (int i = 0; i < PANDA_NUM_MOVABLE_JOINTS; i++) {
        if (boundsT[i][0] > tau[i] or boundsT[i][1] < tau[i]) {
            double inf = std::numeric_limits<double>::infinity();
            for (int j = 0; j < 2 * PANDA_NUM_JOINTS; j++) {
                q_next[j] = inf;
            }
            return;
        }
    }

    // simulate control
    double rtol = 10E-12;
    odeData *data;
    data->tau = tau;
    double s_time = 0.0;
    std::vector<double> sol;
    int isState = 1;
    LSODA lsoda_temp;
    lsoda_temp.lsoda_update(PandaOde, PANDA_NUM_JOINTS * 2, state, sol, &s_time, duration, &isState, (void *)data, rtol);

    for(int i = 0; i < PANDA_NUM_JOINTS * 2; i++) {
        q_next[i] = sol[i];
    }
}