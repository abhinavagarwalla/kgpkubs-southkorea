#ifndef TGOALIE2_HPP
#define TGOALIE2_HPP

#include <list>
#include "comdef.h"
#include "tactic.h"
#include "skillSet.h"
#include "beliefState.h"
#include "logger.h"
#include "config.h"
#include "geometry.hpp"
#include <algorithm>
#include <fstream>

namespace Strategy
{
  class TGoalie2 : public Tactic
  {
  public:
    TGoalie2(const BeliefState* state, int botID) :
      Tactic(Tactic::Stop, state, botID)
    {
    } // TGoalie

    ~TGoalie2()
    { } // ~TGoalKeeping
    inline bool isActiveTactic(void) const
    {
      return false;
    }

    int chooseBestBot(std::list<int>& freeBots, const Tactic::Param* tParam, int prevID) const
    { 
	  //sleep(3);
	  static int counter = 0 ;
      int minv = *(freeBots.begin());
      int mindis = 1000000000;
      Point2D<int> goalPos(ForwardX(-(HALF_FIELD_MAXX)), 0);
      for (std::list<int>::iterator it = freeBots.begin(); it != freeBots.end(); ++it)
      {
                const int factor = 0.2;
        float perpend_dist = ForwardX(state->homePos[*it].x - ForwardX(-HALF_FIELD_MAXX));
                Vector2D<int> goal;
                goal.x = OUR_GOAL_X;
                goal.y = 0;
                float dist_from_goal = Vector2D<int>::dist(state->homePos[*it],goal) ;  
               
				
                if(dist_from_goal + factor * perpend_dist < mindis)
        {
          mindis = dist_from_goal + factor * perpend_dist;
          minv = *it;
        }
      }
	  //printf(" :: %d ::",minv);
	//if(counter==2)
      //assert(tParam=0);
	//counter++;
	return minv;
    } // chooseBestBot

 
	
	    bool isGoalKeeperInPosition()
    {
      if ((ForwardX(state->homePos[botID].x) >  (-HALF_FIELD_MAXX + GOAL_DEPTH )) &&
          (ForwardX(state->homePos[botID].x) <= (-HALF_FIELD_MAXX + GOAL_DEPTH + BOT_RADIUS*2)) &&
          (state->homePos[botID].y >= OUR_GOAL_MINY - DBOX_HEIGHT) &&
          (state->homePos[botID].y <= (OUR_GOAL_MAXY + DBOX_HEIGHT)))
        return true;
      else
        return false;
    }
	
	bool isAngleSet() {
		return ((state->homeAngle[botID] >= (-PI/2 -PI/6) && state->homeAngle[botID] <= (-PI/2 + PI/6)) ||
				(state->homeAngle[botID] <= (PI/2 + PI/6) && state->homeAngle[botID] >= (PI/2 - PI/6))
				);
	}


//bool approaching_from_corner();



 void execute(const Param& tParam)
  {
      float dist = Vector2D<int>::dist(state->ballPos, state->homePos[botID]);


    Vector2D<int> botDestination ;
   float ang1;
	   if(state->ballVel.x> (-100)) 
		   ang1 = 0;
	   else
			ang1 = atan(state->ballVel.y/state->ballVel.x);
	   //in case of ball traveling directly from the oponent's half 
	   botDestination.y = state->ballPos.y + SGN(state->ballVel.y)*((state->ballPos.x) - (-HALF_FIELD_MAXX + DBOX_WIDTH + BOT_RADIUS*1.6))*tan(ang1) ; //tan(ang1)

		cout << botDestination.y << " " << state->ballVel.x << "vel x " << state->ballVel.y << " " << state->ballPos.x << " " << state->ballPos.y << endl;
	//	if(ang1 != 0)
			botDestination.y = botDestination.y + SGN(botDestination.y - state->homePos[botID].y)*2*BOT_RADIUS;
	 if(botDestination.y >=  OUR_GOAL_MAXY)
		   botDestination.y = OUR_GOAL_MAXY ;
	   if(botDestination.y <= OUR_GOAL_MINY)
		   botDestination.y = OUR_GOAL_MINY;
	//cout << "bot dest y " << botDestination.y << endl;
	botDestination.x = (-HALF_FIELD_MAXX + GOAL_DEPTH+ 1.6*BOT_RADIUS); //+ 100;   //set your threshold ********
	// botDestination.y + =  oscillation()*BOT_RADIUS ;// set according to you decide to put oscillation at normal point********

	if (!isGoalKeeperInPosition() && dist > 0.5 * BOT_BALL_THRESH)
      {
        sID = SkillSet::DWGoToPoint;
        sParam.DWGoToPointP.finalSlope =- PI / 2;
        sParam.DWGoToPointP.x = ForwardX(-HALF_FIELD_MAXX + GOAL_DEPTH + BOT_RADIUS*1.6) ;
        sParam.DWGoToPointP.y = botDestination.y;
        sParam.DWGoToPointP.finalVelocity = 0;
		skillSet->executeSkill(sID, sParam);
		return;
		}
		if (!isAngleSet()) {
			sID = SkillSet::TurnToAngle;
			sParam.TurnToAngleP.finalslope = -PI / 2;
			skillSet->executeSkill(sID, sParam);
			return;
		}
					sID = SkillSet::DWGoToPoint;
						sParam.DWGoToPointP.x = ForwardX(-HALF_FIELD_MAXX + GOAL_DEPTH + BOT_RADIUS*1.6);//botDestination.x;
						sParam.DWGoToPointP.y = botDestination.y;
						sParam.DWGoToPointP.finalSlope = -PI/2;
						sParam.DWGoToPointP.finalVelocity = 0;
						skillSet->executeSkill(sID, sParam);  // PI    //set ur angle ******at PI :: to much of disturbation , -PI/2 :: gap is being left ::@oscillation
   // _goToPoint(botID,point,0,PI/2);

  // if(float distance = Vector2D<int>::dist(state->ballPos,state->homePos[botID])< 3.2*BOT_BALL_THRESH) //1.2

}
 
  };// class TGoalKeepingOurside
} // namespace Strategy

#endif // TGOALKEEPINGOUR_HPP
