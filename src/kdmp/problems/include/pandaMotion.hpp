#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "kdmp/spaces/include/pandaControlTorque.hpp"
#include "kdmp/spaces/include/pandaConstants.hpp"

class PandaMotionProblem
{
  public:
    PandaMotionProblem(std::vector<std::vector<float>> stateBounds,
        int stateSpaceSize=12);

    std::shared_ptr<PandaControlSpace> controlSpace;
    std::shared_ptr<omplBase::RealVectorStateSpace> stateSpace;
    std::shared_ptr<omplBase::SpaceInformation> spaceInfo;

};