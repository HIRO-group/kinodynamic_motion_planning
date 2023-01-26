#include <kdmp/problems/include/pandaProblem.hpp>
#include <kdmp/spaces/include/pandaVelCtl.hpp>


namespace omplBase = ompl::base;

class PandaMPWithOptimization : public PandaProblem
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

