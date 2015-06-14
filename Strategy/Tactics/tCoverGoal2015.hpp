#ifndef TTCoverGoal2015_HPP
#define TTCoverGoal2015_HPP

#include <list>
#include "beliefState.h"
#include "tactic.h"
#include "skillSet.h"
#include "config.h"
#include "logger.h"
#include "comdef.h"

#define DEBUG 1
namespace Strategy{
class TCoverGoal2015 : public Tactic{
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
    TCoverGoal2015(const BeliefState* state, int botID) :
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

    ~TCoverGoal2015()
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


    void execute(const Param& tParam)
    {
		cout<< "\n kj\n \n"<<iState<<"\n";
	float distBotBall = Vector2D<int>::dist(state->ballPos,state->homePos[botID]);
	int oppInRegion[4]={0},minRegion=0,targetX = INT_MAX,targetY =-SGN(state->ballPos.y)*BOT_RADIUS;
	float ang1 = atan(state->ballVel.y/state->ballVel.x);
	float goalAngR,goalAngL;
	int yL,yR;
	int predictPosX = ForwardX(-(HALF_FIELD_MAXX-GOAL_DEPTH)*3/4);
	
	goalAngR = atan((state->ballPos.y - OUR_GOAL_MAXY)/(state->ballPos.x + ForwardX(HALF_FIELD_MAXX - GOAL_DEPTH)));
	yR = state->ballPos.y - (state->ballPos.x -predictPosX)*tan(goalAngR);
	
	goalAngL = atan((state->ballPos.y - OUR_GOAL_MINY)/(state->ballPos.x + ForwardX(HALF_FIELD_MAXX - GOAL_DEPTH)));
	yL = state->ballPos.y - (state->ballPos.x - predictPosX)*tan(goalAngL);
	
	int predictPosY = state->ballPos.y - (state->ballPos.x - predictPosX)*tan(ang1);
	int goaliePredictPosY = 0;

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
	

	switch (iState)
	{
	case DEFENDING :
		if( distBotBall < 1.1*BOT_RADIUS )
		{
			iState = CLEARING;
			break;
		}
		if( state->ballPos.x > state->homePos[botID].x )
		{
			if(predictPosY > yR && predictPosY < yL)
			{
				sParam.DWGoToPointP.x = predictPosX ;
				if(  1)//state->homePos[0].y > goaliePredictPosY )
				{
					sParam.DWGoToPointP.y = predictPosY ;//- BOT_RADIUS;
					if( sParam.DWGoToPointP.y < OUR_GOAL_MINY )
						sParam.DWGoToPointP.y = OUR_GOAL_MINY + BOT_RADIUS;
					sParam.DWGoToPointP.finalSlope = PI/2;
					skillSet->executeSkill(SkillSet::DWGoToPoint, sParam);
				}
				else
				{
					sParam.DWGoToPointP.y = predictPosY + BOT_RADIUS;
					if( sParam.DWGoToPointP.y > OUR_GOAL_MAXY )
						sParam.DWGoToPointP.y = OUR_GOAL_MAXY - BOT_RADIUS;
					sParam.DWGoToPointP.finalSlope = PI/2;
					skillSet->executeSkill(SkillSet::DWGoToPoint, sParam);
				}
				

			}
			else if( predictPosY > yR)
			{
				sParam.DWGoToPointP.x = predictPosX ;
				sParam.DWGoToPointP.y = yR - BOT_RADIUS  ;
				sParam.DWGoToPointP.finalSlope = PI/2;
				skillSet->executeSkill(SkillSet::DWGoToPoint, sParam);
			}
			else
			{
				sParam.DWGoToPointP.x = predictPosX ;
				sParam.DWGoToPointP.y = yL + BOT_RADIUS  ;
				sParam.DWGoToPointP.finalSlope = PI/2;
				skillSet->executeSkill(SkillSet::DWGoToPoint, sParam);
			}
		}
		else
		{
			
			iState = BLOCKING ;
		}

		break;
	case CLEARING :
		if( distBotBall > 1.1*BOT_RADIUS)
		{
			iState = DEFENDING;
			break;
		}
		else if( 0)//fabs(normalizeAngle(state->homeAngle[botID] - atan2(state->homePos[botID].y - state->ballPos.y, state->homePos[botID].x - state->ballPos.x))) < PI /4  && state->ballPos.x >= state->homePos[botID].x)
			{
				/*if(minRegion ==1)
					sParam.DWGoToPointP.y = -HALF_FIELD_MAXY + HALF_FIELD_MAXY/4;
				else if(minRegion ==2)
					sParam.DWGoToPointP.y = -HALF_FIELD_MAXY/2 + HALF_FIELD_MAXY/4;
				else if(minRegion ==3)
					sParam.DWGoToPointP.y =  HALF_FIELD_MAXY/4;
				else if(minRegion ==4)
					sParam.DWGoToPointP.y = HALF_FIELD_MAXY/2 + HALF_FIELD_MAXY/4;*/
				sParam.DWGoToPointP.x = state->ballPos.x;
				sParam.DWGoToPointP.y = state->ballPos.y;
				sParam.DWGoToPointP.finalSlope = PI/2;//shouldn't it be 0
				skillSet->executeSkill(SkillSet::DWGoToPoint, sParam);
				break;
			}
			
			else //if(fabs(normalizeAngle(state->homeAngle[botID] - atan2(state->homePos[botID].y - state->ballPos.y, state->homePos[botID].x - state->ballPos.x))) < PI / 2 + PI / 10 && fabs(normalizeAngle(state->homeAngle[botID] - atan2(state->homePos[botID].y - state->ballPos.y, state->homePos[botID].x - state->ballPos.x)))  > PI / 2 - PI / 10)
			{
				sParam.DWGoToPointP.x = state->ballPos.x;
				sParam.DWGoToPointP.y = state->ballPos.y;
				sParam.DWGoToPointP.finalSlope = PI/2;//shouldn't it be 0
				skillSet->executeSkill(SkillSet::DWGoToPoint, sParam);
				break;		
			}
				

	case BLOCKING :
		if( state->ballPos.x > -ForwardX((HALF_FIELD_MAXX-GOAL_DEPTH)*3/4) )
		{
			iState = DEFENDING;
			break;
		}
		else if( abs(predictPosY) < OUR_GOAL_MAXY )
		{
			sParam.DWGoToPointP.y = predictPosY +SGN(state->ballVel.y)*3*BOT_RADIUS ;
			//if(abs(sParam.DWGoToPointP.y) > HALF_FIELD_MAXY)
				//sParam.DWGoToPointP.y = SGN(sParam.DWGoToPointP.y)*HALF_FIELD_MAXY - SGN(sParam.DWGoToPointP.y)*BOT_RADIUS;
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