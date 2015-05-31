#include "skillSet.h"
#include "pathPlanners.h"
#include "beliefState.h"
#include "config.h"
#include "logger.h"
#define GOAL_Y 240
#define AWAYGOAL_X 640
#define ANGLE_THRESH 0.1
namespace Strategy
{
  void SkillSet::goalKeeping(const SParam &param)
  {
    Vector2D<int> ballFinalpos, botDestination, point;
    ballFinalpos.x = state->ballPos.x+state->ballVel.x;
    ballFinalpos.y = state->ballPos.y+state->ballVel.y;
    botDestination.x = -HALF_FIELD_MAXX + DBOX_WIDTH/2;
    /* Workaround for ball velocity 0*/
    if((ballFinalpos.x - state->ballPos.x)==0) 
    {
      //Util::Logger::toStdOut("******************Ball velocity is zero.\n");
      ballFinalpos.x = -HALF_FIELD_MAXX;
      ballFinalpos.y = 0;
    }

    botDestination.y = (ballFinalpos.y - state->ballPos.y)/(ballFinalpos.x - state->ballPos.x)*(botDestination.x - ballFinalpos.x) + ballFinalpos.y;

    point.x =  botDestination.x;
    point.y = botDestination.y;
    
    /* Set Limits on y to not exceed DBOX Y Limits*/
    if(point.y<OUR_GOAL_MINY)
      point.y = OUR_GOAL_MINY + BOT_RADIUS;
    else if(point.y>OUR_GOAL_MAXY)
      point.y = OUR_GOAL_MAXY - BOT_RADIUS;
      
    /* Set Limits on x to not exceed DBOX X Limits*/
    if(point.x <= -HALF_FIELD_MAXX)
      point.x = -HALF_FIELD_MAXX + BOT_BALL_THRESH;
    else if(point.x > -HALF_FIELD_MAXX + 3*DBOX_WIDTH/4)
      point.x = -HALF_FIELD_MAXX + 3*DBOX_WIDTH/4;
      
    //6Util::Logger::toStdOut("Point.x,Point.y  : %d,%d\n",point.x,point.y);
    /* Move to that point */
    
    _goToPoint(botID,point,0,PI,0);
   // _goToPoint(botID,point,0,PI/2);
   
   float dist = Vector2D<int>::dist(point, state->homePos[botID]);
   float theta = normalizeAngle(Vector2D<int>::angle(point, state->homePos[botID]));
   //phiStar =finalslope;
   /* 
    float profileFactor = MAX_BOT_SPEED;
    if(profileFactor < MIN_BOT_SPEED)
      profileFactor = MIN_BOT_SPEED;
    v_y=profileFactor/2; //overshooting occurs at mak speed
    if(state->homePos[botID].y-point.y>0) v_y=0-v_y;  
    comm.addCircle(point.x, point.y, 50);
    if(dist < BOT_BALL_THRESH)
    {
      //comm.sendCommand(botID, 0, 0, 0, 0, false);
      comm.sendCommand(botID, 0, 0); 
    }*/
  }
}
  
