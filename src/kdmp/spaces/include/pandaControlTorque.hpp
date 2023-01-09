#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/control/ODESolver.h>
#include <ompl/base/StateSampler.h>

// #include "models/include/franka_model.h"
// #include <kdmp/models/include/franka_model.h>
#include "kdmp/models/include/franka_model.h"
#include "kdmp/spaces/include/pandaControlSpace.hpp"

namespace omplControl = ompl::control;
namespace omplBase = ompl::base;

class PandaControlTorque : public PandaControlSpace
{
  public:
    PandaControlTorque(const omplBase::StateSpacePtr &stateSpace) : PandaControlSpace(stateSpace)
    {
    }
    void ODE(const omplControl::ODESolver::StateType& q, const omplControl::Control* c, omplControl::ODESolver::StateType& qdot);
    

};