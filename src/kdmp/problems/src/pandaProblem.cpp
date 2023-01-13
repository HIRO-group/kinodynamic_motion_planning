#include "kdmp/problems/include/pandaProblem.hpp"

PandaProblem::PandaProblem(std::vector<std::vector<float>> stateBounds,
    int stateSpaceSize) : mStateSpaceSize(stateSpaceSize)
{
  assert(stateBounds.size() == stateSpaceSize);
  assert(stateBounds[0].size() == 2);

  mStateSpace = std::make_shared<omplBase::RealVectorStateSpace>(stateSpaceSize);
  omplBase::RealVectorBounds stateSpaceBounds(stateSpaceSize);
  for (int i = 0; i < stateSpaceSize; i++) {
    stateSpaceBounds.setLow(i, stateBounds[i][0]);
    stateSpaceBounds.setHigh(i, stateBounds[i][1]);
  }
  mStateSpace->setBounds(stateSpaceBounds);
  mSpaceInfo = std::make_shared<omplBase::SpaceInformation>(mStateSpace);
}

void PandaProblem::setRandomStartGoal(void)
{
  mStartState = std::make_shared<omplBase::ScopedState<>>(mStateSpace);
  mEndState = std::make_shared<omplBase::ScopedState<>>(mStateSpace);

  mStartState->random();
  mEndState->random();
}

void PandaProblem::initializeProblem()
{
    mProblem = std::make_shared<omplBase::ProblemDefinition>(mSpaceInfo);
    mProblem->setStartAndGoalStates(*mStartState, *mEndState);
}

const std::shared_ptr<omplBase::ProblemDefinition> PandaProblem::getProblem()
{
    return mProblem;
}

const std::shared_ptr<omplBase::SpaceInformation> PandaProblem::getSpaceInfo(void)
{
    return mSpaceInfo;
}