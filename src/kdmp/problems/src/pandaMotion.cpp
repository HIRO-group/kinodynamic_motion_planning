#include "problems/include/pandaMotion.hpp"

PandaMotionProblem::PandaMotionProblem(std::vector<std::vector<float>> stateBounds,
    int stateSpaceSize)
{
  assert(stateBounds.size() == stateSpaceSize);
  assert(stateBounds[0].size() == 2);

  stateSpace = std::make_shared<omplBase::RealVectorStateSpace>(stateSpaceSize);
  omplBase::RealVectorBounds stateSpaceBounds(stateSpaceSize);
  for (int i = 0; i < stateSpaceSize; i++) {
    stateSpaceBounds.setLow(i, stateBounds[i][0]);
    stateSpaceBounds.setHigh(i, stateBounds[i][1]);
  }

  stateSpace->setBounds(stateSpaceBounds);

  controlSpace = std::make_shared<PandaControlTorque>(stateSpace);
  omplBase::RealVectorBounds controlStateBounds(PANDA_NUM_JOINTS);

  for (int i = 0; i < PANDA_NUM_JOINTS; i++) {
    controlStateBounds.setLow(i, PANDA_TORQUE_LIMS[i][0]);
  }
}