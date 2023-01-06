#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "kdmp/spaces/include/pandaConstants.hpp"
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/ScopedState.h>

namespace omplBase = ompl::base;

class PandaMPWithOptimization
{
  public:
    PandaMPWithOptimization(std::vector<std::vector<float>> stateBounds = PANDA_JOINT_LIMS,
        int stateSpaceSize=PANDA_NUM_JOINTS, std::string problemType="random");

    std::shared_ptr<omplBase::RealVectorStateSpace> mStateSpace;
    std::shared_ptr<omplBase::SpaceInformation> spaceInfo;

    std::shared_ptr<omplBase::ScopedState<>> mStartState;
    std::shared_ptr<omplBase::ScopedState<>> mEndState;

  private:
    void setRandomStartGoal(void);
    int mStateSpaceSize;

};

