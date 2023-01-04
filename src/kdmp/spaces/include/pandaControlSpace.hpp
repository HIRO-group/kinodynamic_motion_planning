#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/StateSampler.h>
#include <spaces/include/pandaConstants.hpp>

namespace omplControl = ompl::control;
namespace omplBase = ompl::base;

class PandaControlSpace : public omplControl::RealVectorControlSpace
{
  public:
    PandaControlSpace(const omplBase::StateSpacePtr &stateSpace) : omplControl::RealVectorControlSpace(stateSpace, PANDA_NUM_JOINTS)
    {
    }
};