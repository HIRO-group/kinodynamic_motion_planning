#include <kdmp/spaces/include/pandaStateSpace.hpp>

namespace omplBase = ompl::base;

PandaStateSpace::PandaStateSpace(int numDims) : mNumDims(numDims)
{
    for (int i = 0; i < numDims; i++) {
        if (i < PANDA_NUM_JOINTS) {
            bounds_.setLow(i, PANDA_JOINT_LIMS[i][0]);
            bounds_.setHigh(i, PANDA_JOINT_LIMS[i][0]);
        } else {
            int index = i - PANDA_NUM_JOINTS;
            bounds_.setLow(i, PANDA_VEL_LIMS[index][0]);
            bounds_.setHigh(i, PANDA_VEL_LIMS[index][1]);
        }
        
    }
}
