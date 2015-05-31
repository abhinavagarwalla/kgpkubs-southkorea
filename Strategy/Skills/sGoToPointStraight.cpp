#include "skillSet.h"
#include "pathPlanners.h"
#include "beliefState.h"
#include "config.h"
#include "logger.h"
#include <math.h>
#include <stdio.h>

namespace Strategy
{
  #define BOT_POINT_ANGLE_THRESHOLD 0.2
void SkillSet::goToPointStraight(const SParam &param)
	{
		Vector2D<int> dest;
    dest.x = param.GoToPointP.x;
    dest.y = param.GoToPointP.y;
    _goToPointStraight(botID,dest,param.GoToPointP.finalVelocity,param.GoToPointP.finalslope,0.0);
//    float vl = 0,vr = 0;
//    float dist = Vector2D<int>::dist(dest, state->homePos[botID]);   
//		float theta = normalizeAngle(Vector2D<int>::angle(dest, state->homePos[botID]));
//    float init_angle = firaNormalizeAngle(theta - state->homeAngle[botID]);
//    //Util::Logger::toStdOut("BotPos  : ( %d, %d )\n",state->homePos[botID].x, state->homePos[botID].y);
//    Util::Logger::toStdOut("Dist  : %f, Init_Angle  : %f\n",dist,init_angle);
//    if(dist > BOT_BALL_THRESH && fabs(init_angle) > 2*BOT_POINT_ANGLE_THRESHOLD)
//    {
//      /* Rotate in place to move forward.*/
//      //_turnToAngle(init_angle,&vl,&vr);
//      Util::Logger::toStdOut("Turning in place to : %f\n",theta);
//      SParam param2;
//      param2.TurnToAngleP.finalslope = theta;
//      turnToAngle(param2);
//      return;
//    }
//    else if(dist > BOT_BALL_THRESH && fabs(init_angle) <= 2*BOT_POINT_ANGLE_THRESHOLD)
//    {
//      /* Bot aligned correctly. Move forward */
//      Util::Logger::toStdOut("Moving forward by : %f\n",MAX_BOT_SPEED);
//      float profileFactor = (dist < BOT_BALL_THRESH)? 0.6 : dist/(HALF_FIELD_MAXX/2);
//      if(profileFactor > 1) profileFactor = 1.0;
//      float v = profileFactor*MAX_BOT_SPEED;
//      if( fabs(state->homeAngle[botID]-Vector2D<int>::angle(dest,state->homePos[botID])) > 5*BOT_POINT_ANGLE_THRESHOLD)
//        vl = vr = v<5*MIN_BOT_SPEED ? -5*MIN_BOT_SPEED : -v;
//      else
//        vl = vr = v<5*MIN_BOT_SPEED ? 5*MIN_BOT_SPEED : v;
//        
//      comm.addCircle(dest.x, dest.y, 50);
//      comm.addLine(state->homePos[botID].x, state->homePos[botID].y, dest.x, dest.y);
//    }
//    else if(dist < BOT_BALL_THRESH && param.GoToPointP.align)
//    {
//      /* Bot has reached desired location.*/
//      float final_angle = normalizeAngle(param.GoToPointP.finalslope - state->homeAngle[botID]);
//      if(fabs(final_angle) > 0.1)
//      {
//        /*Bot is not aligned to final slope angle. Turn it.*/
//        //_turnToAngle(final_angle,&vl,&vr);
//        SParam param2;
//        param2.TurnToAngleP.finalslope = param.GoToPointP.finalslope;
//        turnToAngle(param2);
//        return;
//      }
//      else
//      {
//        /* Bot is aligned to final slope angle. Stop */
//         Util::Logger::toStdOut("Bot has reached destination.");
//         vl = vr = 0;
//      }
//    }
//    else
//    {
//      Util::Logger::toStdOut("No align required.");
//      vl = vr = 0;
//    }
//#ifdef FIRA_COMM
//      comm.sendCommand(botID, vr,vl);        
//#else
//      comm.sendCommand(botID, vl,vr);
//#endif 
	}
}
