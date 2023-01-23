#ifndef PANDA_SETUP_
#define PANDA_SETUP_

#include <ompl/control/SimpleSetup.h>

class PandaSetup : public ompl::control::SimpleSetUp
{
    public:
        PandaSetup(const std::string &plannerName, const std::vector<double> &stateVec = std::vector<double>(), bool initControl = false);

        ompl::base::PlannerPtr getConfiguredPlannerInstance(const std::string &plannerName);
        
    private:
        void initialize(const std::string &plannerName, const std::vector<double> &stateVec = std::vector<double>(), bool initControl = false);
};

#endif