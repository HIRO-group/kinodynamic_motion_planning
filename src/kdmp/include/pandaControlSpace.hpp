#ifndef PANDA_CONTROL_SPACE_
#define PANDA_CONTROL_SPACE_

#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/SpaceInformation.h>
#include <ompl/base/StateSampler.h>

#include <pandaConstants.hpp>


typedef enum PandaControlType
{
  VELOCITY_CTL = 0, 
  TORQUE_CTL = 1
};


namespace omplControl = ompl::control;
namespace omplBase = ompl::base;

class PandaControlSampler : public omplControl::ControlSampler
{
  public:
    PandaControlSampler(const omplControl::ControlSpace *space) : 
        omplControl::ControlSampler(space)
    {
    }

    virtual void sample(ompl::control::Control *control);
    
    
    // virtual void sample(ompl::control::Control *control, const ompl::base::State *state);
    // virtual void sampleNext(ompl::control::Control *control, const ompl::control::Control * /* previous */,
    //     const ompl::base::State *state)
    // {
    //     sample(control, state);
    // }

    // virtual void steer(ompl::control::Control *control, const ompl::base::State *state, std::vector<double> x);
};


class PandaControlSpace : public omplControl::RealVectorControlSpace
{
  public:
    PandaControlSpace(PandaControlType controlType = TORQUE_CTL, int numDims = 6);

    virtual ompl::control::ControlSamplerPtr allocDefaultControlSampler() const
    {
        return std::make_shared<PandaControlSampler>(this);
    }

  protected:
    int mNumDims;
    PandaControlType mControlType;
};

#endif