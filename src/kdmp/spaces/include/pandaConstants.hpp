#include <vector>

#define PANDA_CTL_RATE 1000
#define PANDA_NUM_JOINTS 7
#define MAX_PLAN_TIME 180.0

const std::vector<std::vector<float>> PANDA_JOINT_LIMS = {{-2.8973, 2.8973}, {-1.7628, 1.7628}, {-2.8973, 2.8973}, {-3.0718, -0.0698}, {-2.8973, 2.8973}, {-0.0175, 3.7525}, {-2.8973, 2.8973}};

const std::vector<std::vector<float>> PANDA_VEL_LIMS = {{-2.1750, 2.1750}, {-2.1750, 2.1750}, {-2.1750, 2.1750}, {-2.1750, 2.1750}, {-2.6100, 2.6100}, {-2.6100, 2.6100}, {-2.6100, 2.6100}};

const std::vector<std::vector<float>> PANDA_ACC_LIMS = {{-15, 15}, {-7.5, 7.5}, {-10, 10}, {-12.5, 12.5}, {-15, 15}, {-20, 20}, {-20, 20}};

const std::vector<std::vector<float>> PANDA_JERK_LIMS = {{-7500, 7500}, {-3750, 3750}, {-5000, 5000}, {-6250, 6250}, {-7500, 7500}, {-10000, 10000}, {-10000, 10000}};

const std::vector<std::vector<float>> PANDA_TORQUE_LIMS = {{-87, 87}, {-87, 87}, {-87, 87}, {-87, 87}, {-12, 12}, {-12, 12}, {-12, 12}};




