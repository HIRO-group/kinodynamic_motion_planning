#include "kdmp/problems/include/pandaMPWithOptimization.hpp"
#include <string.h>

PandaMPWithOptimization::PandaMPWithOptimization(std::vector<std::vector<float>> stateBounds,
    int stateSpaceSize, std::string problemType) :
  mStateSpaceSize(stateSpaceSize), 
  PandaProblem(stateBounds, stateSpaceSize)
{
  if (problemType == "random") {
    setRandomStartGoal();
  }

  initializeProblem();
}
