#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "pandaConstants.hpp"
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/ScopedState.h>
#include <ompl/base/ProblemDefinition.h>

namespace omplBase = ompl::base;

class PandaProblem
{
    public:
        PandaProblem(std::vector<std::vector<float>> stateBounds = PANDA_JOINT_LIMS,
            int stateSpaceSize=PANDA_NUM_JOINTS);

        const std::shared_ptr<omplBase::ProblemDefinition> getProblem(void);
        const std::shared_ptr<omplBase::SpaceInformation> getSpaceInfo(void);

        std::shared_ptr<omplBase::ScopedState<>> mStartState;
        std::shared_ptr<omplBase::ScopedState<>> mEndState;

  protected:
    void initializeProblem(void);
    void setRandomStartGoal(void);
    int mStateSpaceSize;

    std::shared_ptr<omplBase::RealVectorStateSpace> mStateSpace;
    std::shared_ptr<omplBase::SpaceInformation> mSpaceInfo;
    std::shared_ptr<omplBase::ProblemDefinition> mProblem;

};