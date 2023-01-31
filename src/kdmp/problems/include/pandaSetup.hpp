#ifndef PANDA_SETUP_
#define PANDA_SETUP_

#include <ompl/control/SimpleSetup.h>
#include <kdmp/problems/include/RobotInterface.hpp>
#include <vector>
#include <string>

class PandaSetup : public ompl::control::SimpleSetup
{
    public:
        PandaSetup(const char* plannerName, std::shared_ptr<RobotInterface> robot, std::vector<double> &stateVec);

        ompl::base::PlannerPtr getConfiguredPlannerInstance(const std::string &plannerName);
        
    private:
        std::shared_ptr<RobotInterface> panda_;
        ompl::RNG rng_;
};

#endif