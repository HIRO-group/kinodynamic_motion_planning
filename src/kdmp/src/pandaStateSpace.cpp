#include <pandaStateSpace.hpp>
#include <pandaStatePropogator.hpp>
#include <iostream>
#include <math.h>

namespace omplBase = ompl::base;

PandaStateSpace::PandaStateSpace(int numDims) : mNumDims(numDims), RealVectorStateSpace(numDims)
{
    std::cerr<<"setting up panda state\n";
    for (int i = 0; i < numDims / 2; i++) {
        // if (i < PANDA_NUM_JOINTS - 1) {
        //     std::cerr<<"dim: "<<i<<std::endl;
        //     bounds_.setLow(i, PANDA_JOINT_LIMS[i][0]);
        //     bounds_.setHigh(i, PANDA_JOINT_LIMS[i][1]);
        // } else if (i == PANDA_NUM_JOINTS - 1 or i == 2 * PANDA_NUM_JOINTS -1){
        //     bounds_.setHigh(i, 0.001);
        //     bounds_.setLow(i, -0.001);
        // } else {
        //     std::cerr<<"dim: "<<i<<std::endl;
        //     int index = i - PANDA_NUM_JOINTS;
        //     std::cerr<<"index: "<<index<<std::endl;
        //     bounds_.setLow(i, PANDA_VEL_LIMS[index][0]);
        //     bounds_.setHigh(i, PANDA_VEL_LIMS[index][1]);
        // }
        bounds_.setLow(i, PANDA_JOINT_LIMS[i][0]);
        bounds_.setHigh(i, PANDA_JOINT_LIMS[i][1]);
        bounds_.setLow(i + PANDA_NUM_JOINTS, PANDA_VEL_LIMS[i][0]);
        bounds_.setHigh(i + PANDA_NUM_JOINTS, PANDA_VEL_LIMS[i][1]);
    }
    std::cerr<<"panda stat set up\n";
}