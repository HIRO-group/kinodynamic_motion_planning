#ifndef PANDA_STATE_SPACE_
#define PANDA_STATE_SPACE_

#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <pandaConstants.hpp>
#include <kdmp/models/include/franka_model.h>

class PandaStateSpace : public ompl::base::RealVectorStateSpace
{
    public:
    PandaStateSpace(int numDims = 2 * PANDA_NUM_JOINTS);
    virtual void registerProjections();

    protected:
    int mNumDims;

};

#endif