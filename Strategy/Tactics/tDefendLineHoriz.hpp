#ifndef TDEFENDLINEHORIZ_HPP
#define TDEFENDLINEHORIZ_HPP

#include <list>
#include "comdef.h"
#include "tactic.h"
#include "skillSet.h"
#include "beliefState.h"
#include "logger.h"
#include "config.h"

namespace Strategy
{
  class TDefendLineHoriz : public Tactic 
  {
  public:
    TDefendLineHoriz(const BeliefState* state, int botID) :
    Tactic(Tactic::DefendLineHoriz, state, botID)
    {
      //iState = POSITION;
    } // TDefendLine

    ~TDefendLineHoriz()
    { } // ~TDefendLine
	
	int chooseBestBot(std::list<int>& freeBots, const Tactic::Param* tParam, int prevID) const
    {
      int minv = *(freeBots.begin());
      int mindis = 10000;
      Vector2D<int>target((tParam->DefendLineHorizP.x1+tParam->DefendLineHorizP.x2)/2,(tParam->DefendLineHorizP.y1+tParam->DefendLineHorizP.y2)/2);

      for(std::list<int>::iterator it = freeBots.begin(); it != freeBots.end(); ++it)
      {
        int dis_from_target = Vector2D<int>::dist(state->homePos[*it],target);
        if(dis_from_target < mindis)
        {
          mindis = dis_from_target;
          minv = *it;
        }
      }
      return minv;
    } // chooseBestBot
	//int getBotDPointY(int linex) ;
	 void execute(const Param& tParam)
     {
		  sID = SkillSet::GoToPoint;
		  float linex = tParam.DefendLineHorizP.x1;
          sParam.GoToPointP.x = (tParam.DefendLineHorizP.x1 + tParam.DefendLineHorizP.x2)/2 ;
          int temp = getBotDPointY(linex);
          sParam.GoToPointP.y = temp;
		//printf("\n Predicting point = %d \n",temp);
          sParam.GoToPointP.align = false;
          sParam.GoToPointP.finalslope = -PI / 2;
		
	 } 
	 
	 int getBotDPointY(int linex)
    {
      Vector2D<int> ballFinalpos, botDestination, point;
      int flag=2;
     // balldist = state->ballPos.x;
	//		if(state->ballPos.x>0)
		//		balldist =  
				
      float balldistratio = fabs(state->ballPos.x)/(1*(HALF_FIELD_MAXX-DBOX_WIDTH-6*BOT_RADIUS))<1 ? fabs(state->ballPos.x)/(HALF_FIELD_MAXX-DBOX_WIDTH-6*BOT_RADIUS):1 ;
      
      point.y = balldistratio*SGN(state->ballPos.y)*MIN(fabs(state->ballPos.y), OUR_GOAL_MAXY); 
        printf("\npoint.y = %d\n",point.y);
      /* Workaround for ball velocity 0*/
      if( ( ( fabs(state->ballVel.y) + fabs(state->ballVel.x) < 50) ) || (ForwardX(state->ballVel.x)<0 && ForwardX(state->ballVel.x)>(-50)) )
      {
       if(ForwardX(state->ballPos.x) > ( -HALF_FIELD_MAXX*0.3))
        point.y = 0,flag=0;
      }
      else if(ForwardX(state->ballVel.x)>0 )
      {
        if(ForwardX(state->ballPos.x) > (-HALF_FIELD_MAXX*0.3))
          point.y = 0,flag =0;
      }
      else if (ForwardX(state->ballVel.x) <=(-50) )
      {
        if(ForwardX(state->ballPos.x) > (-HALF_FIELD_MAXX*0.8) )
        point.y = (state->ballVel.y/state->ballVel.x)*(linex - (state->ballPos.x)) + state->ballPos.y,flag = 1;
      }
      if(point.y > BOT_RADIUS) point.y -= 1.5*BOT_RADIUS;
      else if(point.y < -BOT_RADIUS) point.y += 1.5*BOT_RADIUS;
         //point.y = point.y + (state->ballVel.y > 0 ? BOT_RADIUS : -BOT_RADIUS);
        
      /* Set Limits on y to not exceed DBOX Y Limits*/
      if(point.y < OUR_GOAL_MINY + 2*BOT_RADIUS)
       {
         if(point.y >-HALF_FIELD_MAXY)
         point.y = OUR_GOAL_MINY + 2*BOT_RADIUS;
        else
          point.y = 0;
       }
       
      else if(point.y > OUR_GOAL_MAXY - 2*BOT_RADIUS)
      {
         if(point.y < HALF_FIELD_MAXY)
          point.y = OUR_GOAL_MAXY - 2*BOT_RADIUS;
          else
          point.y = 0;
       }
//      if(flag==0)
//      printf("\n Center ");
//      else if(flag==1)
//        printf("\n Predicting");
//        else 
//          printf("\n Follow Ball");
          
      return point.y;
    }
  } ; //class Defendline
} //namespace
#endif // TDEFENDLINE_HPP