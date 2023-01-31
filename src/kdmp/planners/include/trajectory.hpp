#include <vector>

struct RobotState
{
    int mNumJoints;
    std::vector<double> q;
    std::vector<double> qdd;
    std::vector<double> qd;
    double dt;
};

typedef std::vector<RobotState> Trajectory;