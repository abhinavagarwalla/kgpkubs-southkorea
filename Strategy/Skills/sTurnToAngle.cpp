#include "skillSet.h"
#include "beliefState.h"

#include <stdio.h>

#define KD_ANGLE 0.1

#include "config.h"
namespace Strategy
{
  void SkillSet::turnToAngle(const SParam& param)
  {
//    static float count = 0,sum = 0,last10th;
    float vl,vr;
    float finalSlope = param.TurnToAngleP.finalslope;
    float turnAngleLeft = normalizeAngle(finalSlope - state->homeAngle[botID]); // Angle left to turn
//    if(count<11)
//    {
//      if(!count)
//        last10th = turnAngleLeft;
//      sum+=turnAngleLeft;
//      count++;
//    }
    
    if(turnAngleLeft>PI/2||turnAngleLeft<-PI/2)
    {
      if(turnAngleLeft>PI/2)
        turnAngleLeft=turnAngleLeft-PI;
      else
        turnAngleLeft=turnAngleLeft+PI;
    }
    
    //float omega = turnAngleLeft * MAX_BOT_OMEGA/(16*PI); // Speedup turn
    float factor = (turnAngleLeft+KD_ANGLE*(turnAngleLeft))/(PI/2);

    vr = 0.4*MAX_BOT_SPEED*(factor)/(PI/2);

    //vr = -MAX_BOT_SPEED*(turnAngleLeft)/(PI/2);
    vl = -vr;
//    vr = -MAX_BOT_SPEED*turnAngleLeft/(PI/2);
//    vl = -vr;
    //printf("turn angle left: %f\n", turnAngleLeft);
    //printf("State home angle: %f",state->homeAngle[botID]);
    //printf("State Home pos: %d %d", state->homePos[botID].x,state->homePos[botID].y);
    
    if(fabs(turnAngleLeft) > DRIBBLER_BALL_ANGLE_RANGE/2) {
#if FIRA_COMM || FIRASSL_COMM
    comm->sendCommand(botID, vl/2, vr/2);        
#else
    comm->sendCommand(botID, vr/2, vl/2);
#endif
    }else {
      comm->sendCommand(botID, 0, 0);
    }
    
   }
}