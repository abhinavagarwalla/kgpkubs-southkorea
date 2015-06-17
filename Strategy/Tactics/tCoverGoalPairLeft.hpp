#ifndef TTCoverGoalPairLeft_HPP
#define TTCoverGoalPairLeft_HPP

#include <list>
#include "beliefState.h"
#include "tactic.h"
#include "skillSet.h"
#include "config.h"
#include "logger.h"
#include "comdef.h"

#define DEBUG 1
namespace Strategy{
class TCoverGoalPairLeft : public Tactic{
public:
    int state1;
     float movementError[10]; // records for previous 10 frames
    float movementErrorSum;
    int movementErrorIndex;
    Point2D<int> prevBotPos;
    float prevBotAngle;
	enum{
		DEFENDING,
		BLOCKING,
		CLEARING
	}iState;
    TCoverGoalPairLeft(const BeliefState* state, int botID) :
      Tactic(Tactic::Stop, state, botID)
    {
      state1 = 0;
            for(int i=0; i<10; i++)
        movementError[i] = 0;
      movementErrorSum  = 0;
      movementErrorIndex = 0;
      prevBotPos = state->homePos[botID];
      prevBotAngle = state->homeAngle[botID];
	  iState = DEFENDING;
    } // TGoalKeeping

    ~TCoverGoalPairLeft()
    { } // ~TGoalKeeping
    inline bool isActiveTactic(void) const
    {
      return false;
    }

    int chooseBestBot(std::list<int>& freeBots, const Tactic::Param* tParam, int prevID) const
    {
      int minv = *(freeBots.begin());
      int mindis = 1000000000;
      Point2D<int> goalPos(ForwardX(-(HALF_FIELD_MAXX)), 0);
      for (std::list<int>::iterator it = freeBots.begin(); it != freeBots.end(); ++it)
      {
const int factor = 0.2;
        float perpend_dist = ForwardX(state->homePos[*it].x - ForwardX(-HALF_FIELD_MAXX));//state->home_goalpoints[2] is center of our goal
Vector2D<int> goal;
goal.x = OUR_GOAL_X;
goal.y = 0;
float dist_from_goal = Vector2D<int>::dist(state->homePos[*it], goal);
if(dist_from_goal + factor * perpend_dist < mindis)
        {
          mindis = dist_from_goal + factor * perpend_dist;
          minv = *it;
        }
      }
      
      return minv;
    } // chooseBestBot

	
	bool isAngleSet() {
		return ((state->homeAngle[botID] >= (-PI/2 -PI/6) && state->homeAngle[botID] <= (-PI/2 + PI/6)) ||
				(state->homeAngle[botID] <= (PI/2 + PI/6) && state->homeAngle[botID] >= (PI/2 - PI/6))
				);
	}


    void execute(const Param& tParam)//////////////////////////////////
    {
		float distBotBall = Vector2D<int>::dist(state->ballPos,state->homePos[botID]);
	int oppInRegion[4]={0},minRegion=0,targetX = INT_MAX,targetY = 0;//-SGN(state->ballPos.y)*BOT_RADIUS;
	float ang1 = state->ballVel.x == 0? PI/2 :atan(state->ballVel.y/state->ballVel.x);
	float goalAngR,goalAngL,divisor;
	int yL,yR;
	

	int predictPosY = state->ballPos.y - (state->ballPos.x - sParam.DWGoToPointP.x)*tan(ang1);
	if(predictPosY >OUR_GOAL_MAXY)
		predictPosY = OUR_GOAL_MAXY - 2*BOT_RADIUS;
	else if( predictPosY < OUR_GOAL_MINY)
		predictPosY = OUR_GOAL_MINY + 2*BOT_RADIUS;
	int goaliePredictPosY = state->ballPos.y - (state->ballPos.x + ForwardX((HALF_FIELD_MAXX-GOAL_DEPTH-BOT_RADIUS)))*tan(ang1);

	Vector2D<int> dest;
	dest.x = ForwardX(-(HALF_FIELD_MAXX-GOAL_DEPTH)*3/4);

	divisor = state->ballPos.x - ForwardX(-HALF_FIELD_MAXX + GOAL_DEPTH);
	goalAngL = divisor == 0?  PI/2 : atan((state->ballPos.y - OUR_GOAL_MAXY)/(divisor));
	goalAngR =divisor == 0?  PI/2 : atan((state->ballPos.y - OUR_GOAL_MINY)/(divisor));
	yL = state->ballPos.y - (state->ballPos.x - dest.x)*tan(goalAngL);
	yR = state->ballPos.y - (state->ballPos.x -dest.x)*tan(goalAngR);
	if(yL > OUR_GOAL_MAXY - BOT_RADIUS)
		yL = OUR_GOAL_MAXY - BOT_RADIUS;
	else if(yR < OUR_GOAL_MINY + BOT_RADIUS)
		yR = OUR_GOAL_MINY + BOT_RADIUS;

	/*for( int i=0;i< 5;i++)
	{
		
		if( state->awayPos[i].x >sParam.DWGoToPointP.x-ForwardX(BOT_RADIUS))
		{
			if( abs(state->awayPos[i].x -  sParam.DWGoToPointP.x) < targetX)
			{
				targetY = state->awayPos[i].y;
				targetX = abs(state->awayPos[i].x -  sParam.DWGoToPointP.x);
			}
		}
	}*/


	//cout<< " here"<<iState<<std::endl;
	switch (iState)
	{
		
	case DEFENDING :
		//cout<< " here"<<iState<<std::endl;
		if( distBotBall < 1.1*BOT_RADIUS )
		{
			iState = CLEARING;
			break;
		}
		if( state->ballPos.x > dest.x)
		{
			if(state->ballVel.x > 0)
			{
				if(state->ballPos.y > OUR_GOAL_MAXY)
					dest.y = OUR_GOAL_MAXY -BOT_RADIUS;
				else if(state->ballPos.y < OUR_GOAL_MINY)
					dest.y = OUR_GOAL_MINY + 3*BOT_RADIUS;
				else
					dest.y = state->ballPos.y +BOT_RADIUS;
			}
			else 
			{
				dest.y = predictPosY ;//-1.2*BOT_RADIUS;
				//cout<< " here"<<std::endl;
			}
		}
		else
		{
			
			iState = BLOCKING ;
			break;
		}
		if(abs(dest.y-state->homePos[botID].y)<0.3*BOT_RADIUS)
			break;
		sParam.DWGoToPointP.x = dest.x;
		sParam.DWGoToPointP.y = dest.y;
		skillSet->executeSkill(SkillSet::DWGoToPoint, sParam);
		break;
	case CLEARING :
		if( distBotBall > 1.1*BOT_RADIUS)
		{
			iState = DEFENDING;
			break;
		}
		else 
		{
				
				sParam.DWGoToPointP.x = state->ballPos.x;
				sParam.DWGoToPointP.y = state->ballPos.y;
				skillSet->executeSkill(SkillSet::DWGoToPoint, sParam);
				break;
		}
			

	case BLOCKING :
		if( state->ballPos.x > -ForwardX((HALF_FIELD_MAXX-GOAL_DEPTH)*3/4) )
		{
			iState = DEFENDING;
			break;
		}
		else if( abs(predictPosY-targetY) < 2*BOT_RADIUS)
		{
			sParam.DWGoToPointP.y = (predictPosY+targetY)/2 -BOT_RADIUS;
		}
		else if( predictPosY < targetY)
		{
			sParam.DWGoToPointP.y = predictPosY +SGN(state->ballVel.y)*1.5*BOT_RADIUS ;
			sParam.DWGoToPointP.finalSlope = PI/2;
			skillSet->executeSkill(SkillSet::DWGoToPoint, sParam);
			break;
		}
		else
		{
			sParam.DWGoToPointP.y = targetY ;
			sParam.DWGoToPointP.finalSlope = PI/2;
			skillSet->executeSkill(SkillSet::DWGoToPoint, sParam);
			break;
		}
		
	}
	}


}; // class TDefense
} // namespace Strategy

#endif // TTCharge_HPP