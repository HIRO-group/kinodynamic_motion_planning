#include <ompl/geometric/planners/rrt/RRTConnect.h>

namespace omplGeo = ompl::geometric;

class RRTConnectTimePara
{
    RRTConnectTimePara::RRTConnectTimePara();

    void performTimeParameterization();
    
    std::shared_ptr<omplGeo::RRTConnect> configPlanner;
    // need optimizer for time parameterization

};