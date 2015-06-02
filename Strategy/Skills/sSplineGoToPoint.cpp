#include "skillSet.h"
#include "beliefState.h"
#include "logger.h"
#include "config.h"
#include "pose.h"
#include <iostream>

using namespace Util;

namespace Strategy
{
  void SkillSet::splineGoToPoint(const SParam& param)
  {
	  Vector2D<int> dpoint;
    float finalvel;
    finalvel  = param.GoToPointP.finalVelocity;
	Pose start(state->homePos[botID].x, state->homePos[botID].y, state->homeAngle[botID]);
	Pose end(param.SplineGoToPointP.x, param.SplineGoToPointP.y, param.SplineGoToPointP.finalslope);
	
	_splineGoToPoint(botID, start, end, finalvel);
	
   }
}