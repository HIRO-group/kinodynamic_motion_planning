#ifndef INTERFACE_
#define INTERFACE_

#include "vector"
#include "Eigen/Core"

class RobotInterface
{
    public:
    RobotInterface() {}
    virtual std::vector<double> inverseKinematics(std::vector<double> eePose) = 0;
    virtual std::vector<double> forwardKinematics(std::vector<double> q) = 0;
    virtual bool inCollision(std::vector<double> q) = 0;
    virtual void setRobotState(std::vector<double> state) = 0;
    virtual void sendControlCommand(std::vector<double> controlCommand) = 0;
    virtual std::vector<double> getRandomConfig(void) = 0;
    virtual std::vector<double> eeVelToJointVel(std::vector<double> eeVel, std::vector<double> q) = 0;
    virtual std::vector<double> jointVelToEeVel(std::vector<double> qd, std::vector<double> q) = 0;

    virtual Eigen::MatrixXd getJacobian(std::vector<double> q) = 0;

    virtual std::vector<double> ddqFromEEAcc(std::vector<double> ee_acc, std::vector<double> dq, std::vector<double> q) = 0;

    virtual std::vector<double> sampleControl() = 0;

};

#endif