#include "pandaStatePropogator.hpp"
#include "pandaStateSpace.hpp"
#include "pandaMoveitInterface.hpp"
#include "kdmpUtils.hpp"

#include "kdmp_ros/PandaControlCmd.h"

#include<unistd.h> 

void propagate(std::vector<double> start, const std::vector<double> control,
                const double duration, std::vector<double> &result, 
                std::shared_ptr<PandaMoveitInterface> panda_)
{
    const double *t = control.data();
    double *x = start.data();
    double *q_next = result.data();
    std::vector<double> eeVel(t, t + 6);
    for (int j = 0; j < (duration / .001); j ++) {
        std::vector<double> state(x, x + 2 * PANDA_NUM_JOINTS);
        std::vector<double> q(x, x+PANDA_NUM_JOINTS);
        std::vector<double> dq = panda_->eeVelToJointVel(eeVel, q);
        std::vector<double> ee_acc(6, 0.0);
        printVec(dq, "Calculated new joint velocity: ");
        std::vector<double> ddq(panda_->ddqFromEEAcc(ee_acc, dq, q));
        printVec(ddq, "Calculated ddq: ");
        std::vector<std::vector<double>> boundsA = PANDA_ACC_LIMS;
        for (int i = 0; i < PANDA_NUM_MOVABLE_JOINTS; i++) {
            if (boundsA[i][0] > ddq[i] or boundsA[i][1] < ddq[i]) {
                double inf = std::numeric_limits<double>::infinity();
                for (int j = 0; j < 2 * PANDA_NUM_JOINTS; j++) {
                    q_next[j] = inf;
                }
                std::cout<<" ACC out of bounds!\n";
                return;
            }
        }
        std::vector<double> tau = get_tau(q, dq, ddq);
        printVec(tau, "Calculated tau: ");
        auto boundsT = PANDA_TORQUE_LIMS;
        for (int i = 0; i < PANDA_NUM_MOVABLE_JOINTS; i++) {
            if (boundsT[i][0] > tau[i] or boundsT[i][1] < tau[i]) {
                double inf = std::numeric_limits<double>::infinity();
                for (int j = 0; j < 2 * PANDA_NUM_JOINTS; j++) {
                    q_next[j] = inf;
                }
                std::cout<<"TAU out of bounds!\n";
                return;
            }
        }
        // simulate control
        double rtol = 10E-12;
        odeData data;
        for (auto &t : tau) {
            data.tau.push_back(t);
        }
        double s_time = 0.0 + (.001 * j);
        double end_time = 0.0 + (0.001 * (j + 1));
        std::vector<double> sol;
        int isState = 1;
        LSODA lsoda_temp;
        std::stringstream ss;
        lsoda_temp.lsoda_update(PandaOde, PANDA_NUM_JOINTS * 2, state, sol, &s_time, end_time, &isState, (void *)&data, rtol);
        isState = 2;
        printVec(std::vector<double>(sol.begin() + 1, sol.begin() + 8), "Calclated q_end: ");
        printVec(std::vector<double>(sol.begin() + 8, sol.end()), "Calclated qd_end: ");
        if (j == (duration / 0.001) - 1) {
            for(int i = 0; i < PANDA_NUM_JOINTS * 2; i++) {
                q_next[i] = sol[i+1];
            }
        } else {
            for(int i = 0; i < PANDA_NUM_JOINTS * 2; i++) {
                x[i] = sol[i+1];
            }
        }
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "test_propogate_node");
    ros::NodeHandle nh;
    std::shared_ptr<PandaMoveitInterface> panda_ = 
            std::make_shared<PandaMoveitInterface>(nh);
    std::cout << "\n\n\n made panda interface\n";
    kdmp_ros::PandaControlCmd test_cmd;
    sleep(1);
    test_cmd.tau = {-0.68194838, -37.73460385, 16.70265844, 8.21490198, 0.7144556, 0.33786598, 0.19650288};
    test_cmd.q_start = {-1.20341642,  1.1850724,   0.9988895,  -1.46191202,  2.72541572,  3.26651575,
        -2.29260979};
    test_cmd.q_end = {-1.21022686,  1.16758441,  0.97319709, -1.54278718,  2.80661078,  3.15263343,
        -2.24018816};
    test_cmd.qd_start = {0.22662593, -0.28590787,  1.05141934,  0.29856902,  2.07664398, -1.66599341,
        1.66131537};
    test_cmd.qd_end = {-0.11965429, -0.06310297, -0.5232782,  -0.99595745,  0.46659824, -0.95971936,
        0.16730152};
    test_cmd.durration = 0.02;
    std::vector<double> ee_vel_cmd = {-0.1815400590680458, 0.14613396456822683, -0.3630013236511493, 0.44895068498079, 0.23023960442418578, -0.4324083382923338};
    std::cout << "\n\n\n \n";
    std::vector<double> q_end(14), q_start = test_cmd.q_start;
    q_start.insert(q_start.end(), test_cmd.qd_start.begin(), test_cmd.qd_start.end());
    propagate(q_start, ee_vel_cmd, test_cmd.durration, q_end, panda_);
    std::cout << "\n\n\n propogated \n";

    printVec(test_cmd.q_end, "Given final q: ");
    printVec(test_cmd.tau, "Given Tau: ");
    printVec(test_cmd.qd_end, "Given qd_end: ");
    
    return 0;
}