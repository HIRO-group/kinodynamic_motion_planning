#ifndef INTERFACE_
#define INTERFACE_

#include "vector"

class RobotInterface
{
    public:
    RobotInterface() {}
    virtual std::vector<double> inverseKinematics(std::vector<double> eePose) = 0;
    virtual bool inCollision(std::vector<double> q) = 0;
    virtual void setRobotState(std::vector<double> state) = 0;
    virtual void sendControlCommand(std::vector<double> controlCommand) = 0;
    virtual std::vector<double> getRandomConfig(void) = 0;

};

#endif