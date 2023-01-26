#ifndef PANDA_SETUP_
#define PANDA_SETUP_

#include <ompl/control/SimpleSetup.h>
#include <kdmp/problems/include/RobotInterface.hpp>

class PandaSetup : public ompl::control::SimpleSetup
{
    public:
        PandaSetup(const std::string &plannerName, RobotInterface* robot, const std::vector<double> &stateVec = std::vector<double>());

        ompl::base::PlannerPtr getConfiguredPlannerInstance(const std::string &plannerName);
        std::shared_ptr<RobotInterface> panda;
        
    private:
        void initialize(const std::string &plannerName, const std::vector<double> &stateVec = std::vector<double>(), bool initControl = false);
        std::shared_ptr<RobotInterface> panda_;
        ompl::RNG rng_;
};

#endif