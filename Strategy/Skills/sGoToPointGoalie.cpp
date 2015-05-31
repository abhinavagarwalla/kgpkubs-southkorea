#include "skillSet.h"
#include "pathPlanners.h"
#include "beliefState.h"
#include "config.h"
#include "logger.h"
#include <math.h>
#include <stdio.h>
#include "logger.h"
namespace Strategy
{
  void SkillSet::goToPointGoalie(const SParam &param)
  {
    Vector2D<int> dpoint;
    float finalvel;
    dpoint.x  = param.GoToPointP.x;
    dpoint.y  = param.GoToPointP.y;
    finalvel  = param.GoToPointP.finalVelocity;
    //Util::Logger::toStdOut("Going to point : (%d , %d) from (%d, %d)",dpoint.x,dpoint.y, state->homePos[botID].x, state->homePos[botID].y);
    float dist = (state->homePos[botID] - Vector2D<int>(dpoint)).absSq();
    //if(goalie==true)
    if(!param.GoToPointP.align )//|| dist < 9 * BOT_BALL_THRESH * BOT_BALL_THRESH)
      _goToPointLessThresh(botID, dpoint, finalvel, param.GoToPointP.finalslope, 0,param.GoToPointP.increaseSpeed);
    else
      _goToPointLessThresh(botID, dpoint, finalvel, param.GoToPointP.finalslope,0,param.GoToPointP.increaseSpeed);
  }
}
