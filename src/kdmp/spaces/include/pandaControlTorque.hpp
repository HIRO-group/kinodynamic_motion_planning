#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/StateSampler.h>

#include <spaces/include/pandaControlSpace.hpp>

namespace omplControl = ompl::control;
namespace omplBase = ompl::base;

class PandaControlTorque : public PandaControlSpace
{
  public:
    PandaControlTorque(const omplBase::StateSpacePtr &stateSpace) : PandaControlSpace(stateSpace)
    {
    }

};