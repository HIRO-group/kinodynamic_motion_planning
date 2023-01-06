#include "problems/include/pandaMotion.hpp"
#include <string.h>

PandaMPWithOptimization::PandaMPWithOptimization(std::vector<std::vector<float>> stateBounds,
    int stateSpaceSize, std::string problemType) : mStateSpaceSize(stateSpaceSize)
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

  if (problemType.compare("random")== 0) {
    setRandomStartGoal();
  }
}

void PandaMPWithOptimization::setRandomStartGoal(void)
{
  mStartState = std::make_shared<omplBase::ScopedState<>>(mStateSpace);
  mEndState = std::make_shared<omplBase::ScopedState<>>(mStateSpace);

  mStartState->random();
  mEndState->random();
}
