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
		BOTH_BLOCKING ,
		CLEARING,
		ATTACKING
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
   
    // change choose best bot based on the left condition 
				//Commented. Test remaining
	 int chooseBestBot(std::list<int>& freeBots, const Tactic::Param* tParam, int prevID) const
		{
			int minv = *(freeBots.begin());
			int minv1, minv2;
			int mindis = 1000000000;
			int mindis_t = 1000000000;
			int it_t = -1 ; 
			Point2D<int> goalPos(ForwardX(-(HALF_FIELD_MAXX)), 0);
			for (std::list<int>::iterator it = freeBots.begin(); it != freeBots.end(); ++it)
			{
				
				float botBallDist = 1000000000;
				float botBallDist_t = 1000000000;
				const int factor = 0.2;
				float perpend_dist = ForwardX(state->homePos[*it].x - ForwardX(-HALF_FIELD_MAXX));//state->home_goalpoints[2] is center of our goal
				Vector2D<int> goal;
				goal.x = OUR_GOAL_X;
				goal.y = 0;
				float dist_from_goal = Vector2D<int>::dist(state->homePos[*it], goal);
				if(dist_from_goal + factor * perpend_dist < mindis)
				{
					mindis_t = mindis;
					mindis = dist_from_goal + factor * perpend_dist;
					minv2 = minv1;
					minv1 = *it;
				}
				else if(dist_from_goal + factor*perpend_dist < mindis_t)
				{
					mindis_t = dist_from_goal + factor * perpend_dist;
					minv2 = *it;
				}
				if(state->ballPos.x < ForwardX(-(HALF_FIELD_MAXX-GOAL_DEPTH)*0.8 + 2*BOT_RADIUS) && (state->ballPos.y >OUR_GOAL_MAXY))
				{
					botBallDist = Vector2D<int>::dist(state->ballPos, state->homePos[*it]);
					if(botBallDist < botBallDist_t /*&& state->ballPos.y > OUR_GOAL_MAXY*/ && state->ballPos.y > state->homePos[*it].y && abs(state->ballPos.x - state->homePos[*it].x ) <= 2*BOT_RADIUS)
					{
							botBallDist_t = botBallDist;
							it_t = *it;
					}
					else if( botBallDist < botBallDist_t && state->ballPos.y < OUR_GOAL_MINY && state->ballPos.y < state->homePos[*it].y /*&& abs(state->ballPos.x - state->homePos[it].x ) <= 2*BOT_RADIUS*/)
					{
						botBallDist_t = botBallDist;
						if( minv1 == *it)
							it_t = minv2;
						else if( minv2 == *it)
							it_t = minv1;
					}
				}
			}
			if(it_t == -1)
			{
				if( state->homePos[minv1].y > state->homePos[minv2].y)
					minv = minv1;
				else minv = minv2;
			}
			else return it_t;
			
			return minv;
		} // chooseBestBot
	 
	/*
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
*/
	
	bool isAngleSet() {
		return ((state->homeAngle[botID] >= (-PI/2 -PI/6) && state->homeAngle[botID] <= (-PI/2 + PI/6)) ||
				(state->homeAngle[botID] <= (PI/2 + PI/6) && state->homeAngle[botID] >= (PI/2 - PI/6))
				);
	}
   
   int nearestOppBot(int destx)
   {
	int id = 4 ;
	int minDis = HALF_FIELD_MAXX ;
    int distx ;	
 	for( int i=0;i< 5;i++)
	{
		distx = abs(destx - state->awayPos[i].x) ; 
		if(distx < minDis)
		{
		  	minDis = distx ;
			id  = i ;
		}

	}
	   
	return id ;   
   }

    void execute(const Param& tParam)//////////////////////////////////
    {
	int oppID ;
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
	dest.x = ForwardX(-(HALF_FIELD_MAXX-GOAL_DEPTH)*0.8);

	//cout<< " here"<<iState<<std::endl;
	switch (iState)
	{
		
	case DEFENDING :
	
		if(state->ballPos.x < dest.x && abs(state->ballPos.y) < OUR_GOAL_MAXY)
		{
		   iState = BOTH_BLOCKING ;	
		   break ;
		}
		if(state->ballPos.x < dest.x && (state->ballPos.y < OUR_GOAL_MINY))
		{
		  iState = BLOCKING ;
		  break ;
		}
		if(state->ballPos.x < dest.x + 2*BOT_RADIUS && (state->ballPos.y >OUR_GOAL_MAXY))
		{
		   iState = CLEARING ;
		   break ;
		}
		if( state->ballPos.x > dest.x)
		{
			if(state->ballVel.x > 0 || abs(abs(ang1) - PI/2) < PI/12)
			{
				if(state->ballPos.y > OUR_GOAL_MAXY)
					dest.y = OUR_GOAL_MAXY + BOT_RADIUS;
				else if(state->ballPos.y < OUR_GOAL_MINY)
					dest.y = OUR_GOAL_MINY + 2*BOT_RADIUS;
				else
					dest.y = state->ballPos.y +1*BOT_RADIUS;
			}
			else 
			{
				dest.y = predictPosY + 1*BOT_RADIUS;
				//cout<< " here"<<std::endl;
			}
		}

		if(abs(dest.y-state->homePos[botID].y)<0.3*BOT_RADIUS)
			break;
		sParam.DWGoToPointP.x = dest.x;
		sParam.DWGoToPointP.y = dest.y;
		skillSet->executeSkill(SkillSet::DWGoToPoint, sParam);
		break;
		
	case CLEARING :
		 // CHANGE THE COND. FOR CLEARING .. INSTEAD OF GOTOBALL .. USE PREDICTION OR GO FROM BACK
		if(state->ballPos.x < dest.x && abs(state->ballPos.y) < OUR_GOAL_MAXY)
		{
		   iState = BOTH_BLOCKING ;	
		   break ;
		}
		if(state->ballPos.x > dest.x + 2*BOT_RADIUS )
		{
			iState = DEFENDING;
			break;
		}
		
		if(state->ballPos.x < dest.x && (state->ballPos.y < OUR_GOAL_MINY))
		{
		  iState = BLOCKING ;
		  break ;
		}
		// clear the ball away 
		// currently its just going to the ball .. change the algo..to go from the back 
		        sParam.GoToPointP.x = state->ballPos.x;
				sParam.GoToPointP.y = state->ballPos.y;
				skillSet->executeSkill(SkillSet::GoToPoint, sParam);
				break;
		
			

	case BLOCKING :
	
		if(state->ballPos.x < dest.x && abs(state->ballPos.y) < OUR_GOAL_MAXY)
		{
		   iState = BOTH_BLOCKING ;	
		   break ;
		}
		if(state->ballPos.x > dest.x + 2*BOT_RADIUS )
		{
			iState = DEFENDING;
			break;
		}
		
		if(state->ballPos.x < dest.x && (state->ballPos.y > OUR_GOAL_MAXY))
		{
		   iState = CLEARING ;
		   break ;
		}
		
		oppID = 4 ;//nearestOppBot(dest.x) ;
			std::cout<<"ID = "<<oppID<<std::endl;
			if(abs(state->awayPos[oppID].y) > OUR_GOAL_MAXY)
				sParam.DWGoToPointP.y = SGN(state->awayPos[oppID].y)*OUR_GOAL_MAXY ;
			else
				sParam.DWGoToPointP.y = state->awayPos[oppID].y ;
			sParam.DWGoToPointP.x = dest.x;
			skillSet->executeSkill(SkillSet::DWGoToPoint, sParam);
			break;
			
    case BOTH_BLOCKING :
	    
		if(state->ballPos.x > dest.x + 2*BOT_RADIUS )
		{
			iState = DEFENDING;
			break;
		}
		
		if(state->ballPos.x < dest.x && (state->ballPos.y > OUR_GOAL_MAXY))
		{
		   iState = CLEARING ;
		   break ;
		}
		
		if(state->ballPos.x < dest.x && (state->ballPos.y < OUR_GOAL_MINY))
		{
		  iState = BLOCKING ;
		  break ;
		}
		
		oppID = 4 ;//nearestOppBot(dest.x) ;
			std::cout<<"ID = "<<oppID<<std::endl;
			if(state->awayPos[oppID].y > 0 )
			{
			  if(abs(state->awayPos[oppID].y) > OUR_GOAL_MAXY)
				sParam.DWGoToPointP.y = SGN(state->awayPos[oppID].y)*OUR_GOAL_MAXY ;
			  else
				sParam.DWGoToPointP.y = state->awayPos[oppID].y ;
			}
			else
			  sParam.DWGoToPointP.y = OUR_GOAL_MAXY ;
		    
			sParam.DWGoToPointP.x = dest.x;
			skillSet->executeSkill(SkillSet::DWGoToPoint, sParam);
			break;
	  
	}
  }


}; // class TDefense
} // namespace Strategy

#endif // TTCharge_HPP

