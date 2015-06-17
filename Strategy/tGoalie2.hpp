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
		return ((state->homeAngle[botID] >= (-PI/2 -PI/10) && state->homeAngle[botID] <= (-PI/2 + PI/10)) ||
				(state->homeAngle[botID] <= (PI/2 + PI/10) && state->homeAngle[botID] >= (PI/2 - PI/10))
				);
	}
//bool approaching_from_corner();

 void execute(const Param& tParam)
  {
    Vector2D<int> botDestination ;
   float ang1;
	   if(state->ballVel.x> (-10)) 
		   ang1 = 0;
	   else
			ang1 = atan(state->ballVel.y/state->ballVel.x);
		
		botDestination.y = state->ballPos.y + SGN(state->ballVel.y)*2*BOT_RADIUS  - ((state->ballPos.x)-(-HALF_FIELD_MAXX + DBOX_WIDTH + BOT_RADIUS))*tan(ang1);
 //SGN(state->ballVel.y)*2*BOT_RADIUS 
	//`botDestination.y = state->ballPos.y - ((state->ballPos.x) - (-HALF_FIELD_MAXX + DBOX_WIDTH + BOT_RADIUS*1.6))*ang1 ; //tan(ang1)
		cout << " fdsafw               "  << state->ballVel.x << "   " << state->ballVel.y  << endl;//<< " " << state->ballPos.x << " " << state->ballPos.y << endl;
		botDestination.x = (-HALF_FIELD_MAXX + GOAL_DEPTH+ 1.6*BOT_RADIUS);
		
		float botBallDist = Vector2D<int>::dist(state->ballPos, state->homePos[botID]);
		cout << "vbsfvgiejv             " <<state->ballPos.y << "   " << botDestination.y << endl;
	//	botDestination.y = botDestination.y + SGN(state->ballVel.y)*((2*BOT_RADIUS));
		//if(botBallDist < 1500)
	  //botDestination.y = botDestination.y + SGN(botDestination.y - state->homePos[botID].y)*2*((int)((botBallDist/1500)*BOT_RADIUS));
	//  botDestination.y = botDestination.y + SGN(botDestination.y - state->homePos[botID].y)*2*BOT_RADIUS;
	  float dist = Vector2D<int>::dist(botDestination, state->homePos[botID]);	
	 if(botDestination.y >=  (OUR_GOAL_MAXY - 0.7*BOT_RADIUS))
		   botDestination.y = OUR_GOAL_MAXY - 0.7*BOT_RADIUS;
	   if(botDestination.y <= (OUR_GOAL_MINY + 0.7*BOT_RADIUS))
		   botDestination.y = OUR_GOAL_MINY + 0.7*BOT_RADIUS;
	//cout << "bot dest y " << botDestination.y << endl;
	 //+ 100;   //set your threshold ********
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

}
 
  };// class TGoalKeepingOurside
} // namespace Strategy

#endif // TGOALKEEPINGOUR_HPP
