#include "skillSet.h"

namespace Strategy
{
  void SkillSet::velocity(const SParam& param)
  {
    errorLog(botID,(param.VelocityP.vl),(param.VelocityP.vr), state);
    comm->sendCommand(botID,param.VelocityP.vl,param.VelocityP.vr);
    
    //comm.sendCommand(botID, 0, param.VelocityP.v, param.VelocityP.v_t, 0, false);
  } // velocity
}
