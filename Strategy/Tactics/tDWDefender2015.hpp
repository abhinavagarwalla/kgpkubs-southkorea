#ifndef TDWDEFENDER2015_HPP
#define TDWDEFENDER2015_HPP

#include <list>
#include "comdef.h"
#include "tactic.h"
#include "skillSet.h"
#include "beliefState.h"
#include "logger.h"
#include "config.h"
#include "pose.h"

namespace Strategy
{
    class TDWDefender2015 : public Tactic
    {
		
	public:
		  int hasAchievedOffset;
		
			TDWDefender2015(const BeliefState* state, int botID) :
			  Tactic(Tactic::DWDefender, state, botID)
			{
			  iState = DEFENDING;
			} // TDWDefender	
			~TDWDefender2015()
			{ } // ~TDWDefender

		enum InternalState
		{
		  DEFENDING,
		  SPINNING_CCW = -1,
		  SPINNING_CW = 1,
		  ATTACKING,
		} iState;
		inline bool isActiveTactic(void) const
		{
			return false;
		}
		   //CHOOSEbEST bOT AND the giving of parameters for going to the required point needs to be entered
    //Choose best bot also needs to get the params that the tactic has in order to choose the best bot....

		inline bool isPointinField(const Vector2D<int>& point){
			if(abs(point.x) <= HALF_FIELD_MAXX && abs(point.y) <= HALF_FIELD_MAXY)
				return 1;
			else 
				return 0;
		}
		int chooseBestBot(std::list<int>& freeBots, const Tactic::Param* tParam, int prevID) const
		{
		  assert(tParam != 0);
		  int minv   = *(freeBots.begin());
		  int mindis = 1000000000;
		  Vector2D<int> tGoToPoint(tParam->PositionP.x, tParam->PositionP.y);
		  
		  for (std::list<int>::iterator it = freeBots.begin(); it != freeBots.end(); ++it)
		  {
			// TODO make the bot choosing process more sophisticated, the logic below returns the 1st available bot
			float dis_from_point = (state->homePos[*it] - tGoToPoint).absSq();
			if(dis_from_point < mindis)
			{
			  mindis = dis_from_point;
			  minv = *it;
			}
		  }
		  printf("%d assigned Position\n", minv);
		  return minv;
		} // chooseBestBot
	
    bool pointxInField(Vector2D<int> final)
    {
      if((final.x < HALF_FIELD_MAXX - (BALL_AT_CORNER_THRESH) && final.x > -HALF_FIELD_MAXX + (BALL_AT_CORNER_THRESH)))
      {
        if((final.y < HALF_FIELD_MAXY - BALL_AT_CORNER_THRESH && final.y > -HALF_FIELD_MAXY + BALL_AT_CORNER_THRESH))
        {
          return true;
        }
        else return false;
      }
      else  return false;
    }

void execute(const Param& tParam)
{
			//cout<<"\n \n" <<"istate"<< iState;
			static Vector2D<float> lastVel[10];
			static int index = 0;
			if(index < 10) {
				lastVel[index].x = state->ballVel.x;
				lastVel[index].y = state->ballVel.y;
				index = (index + 1) % 10;
			}
			Vector2D<float> avgBallVel(0.0,0.0);
			for(int i=0;i<10;i++) {
				avgBallVel.x += lastVel[i].x;
				avgBallVel.y += lastVel[i].y;
			}
			avgBallVel.x /= 10.0;
			avgBallVel.y /= 10.0;
			//______________________________________
			
			
	float ang1 = atan(state->ballVel.y/state->ballVel.x);
	float distBotBall = Vector2D<int>::dist(state->ballPos,state->homePos[botID]);
	float distOppBall = Vector2D<int>::dist(state->ballPos,state->awayPos[state->oppBotNearestToBall]);

	switch(iState)
	{
	case DEFENDING :
	//	cout<< 1;
		
		if( distBotBall < 1.1*BOT_RADIUS )//&& abs(abs(state->homeAngle[botID]) - PI/2 ) < PI/12)
		{
			if( fabs(normalizeAngle(state->homeAngle[botID] - atan2(state->homePos[botID].y - state->ballPos.y, state->homePos[botID].x - state->ballPos.x))) < PI /4  && state->ballPos.x >= state->homePos[botID].x)
			{                                      
				sParam.DWGoToPointP.x = state->ballPos.x;
				sParam.DWGoToPointP.y = state->ballPos.y;
				sParam.DWGoToPointP.finalSlope = PI/2;
				skillSet->executeSkill(SkillSet::DWGoToPoint, sParam);
			}
			
			else //if(fabs(normalizeAngle(state->homeAngle[botID] - atan2(state->homePos[botID].y - state->ballPos.y, state->homePos[botID].x - state->ballPos.x))) < PI / 2 + PI / 10 && fabs(normalizeAngle(state->homeAngle[botID] - atan2(state->homePos[botID].y - state->ballPos.y, state->homePos[botID].x - state->ballPos.x)))  > PI / 2 - PI / 10)
				{
					//sID = SkillSet::Spin;
				   if(state->ballPos.x >=state->homePos[botID].x) 
				   {
					
					if(state->ballPos.y > 0)
					{
						//sParam.SpinP.radPerSec = (-MAX_BOT_OMEGA);
						iState = SPINNING_CW;
					}
						
					else
					{
						//sParam.SpinP.radPerSec = (MAX_BOT_OMEGA);
						iState =SPINNING_CCW;
					}
				   }
				}
				break;
		}
		
		else 
		   if(state->ballPos.x > 0)
		   {
			cout << "\n\n\asfhah atrtack\n\n\n" << state->ballPos.x << " " << (-HALF_FIELD_MAXX/4) << endl;
			int predictPosX = ForwardX(-(HALF_FIELD_MAXX/2)) ;
			int predictPosY = state->ballPos.y - (state->ballPos.x - predictPosX)*tan(ang1);
			predictPosY = predictPosY + SGN(predictPosY - state->homePos[botID].y)*2*BOT_RADIUS;
			if(state->ballVel.x < 0){
				sParam.DWGoToPointP.x = predictPosX;
				
				if(predictPosY > HALF_FIELD_MAXY - BOT_RADIUS)
					predictPosY = HALF_FIELD_MAXY - 2*BOT_RADIUS;
				else if( predictPosY < (-HALF_FIELD_MAXY + BOT_RADIUS))
					predictPosY = (-HALF_FIELD_MAXY + 2*BOT_RADIUS);
					
				sParam.DWGoToPointP.y = predictPosY ; 
				sParam.DWGoToPointP.finalSlope = PI/2;
				skillSet->executeSkill(SkillSet::DWGoToPoint, sParam);
			}
			else{
				sParam.DWGoToPointP.x = predictPosX ;
				sParam.DWGoToPointP.y = state->ballPos.y  ;
				sParam.DWGoToPointP.finalSlope = PI/2;
				skillSet->executeSkill(SkillSet::DWGoToPoint, sParam);
			}
			break;
		   }
		   else 
			 if(state->ballPos.x < 0 && state->ballPos.x > state->homePos[botID].x)
		     {
			  int predictX = -(HALF_FIELD_MAXX/2) ;
			  int predictY = state->ballPos.y + (state->ballPos.x - predictX )*tan(ang1) ;
			  predictY = predictY - SGN(predictY - state->homePos[botID].y)*2*BOT_RADIUS;
			  distOppBall = HALF_FIELD_MAXX ; // for testing as there is no opponent bot to test 
			  if((distOppBall < 2*BOT_BALL_THRESH))
			  {
				sParam.DWGoToPointP.x = predictX;
				sParam.DWGoToPointP.y = predictY  ;
				sParam.DWGoToPointP.finalSlope = PI/2;
				skillSet->executeSkill(SkillSet::DWGoToPoint, sParam);
				break;
			  }
			  else 
			  {
				iState =  ATTACKING;
				break;
			  }
			 }
			 else
			{ 
		      if (state->ballPos.x <= 0 && state->ballPos.x <= state ->homePos[botID].x && state->ballPos.x>=-(HALF_FIELD_MAXX/*-1.2*DBOX_WIDTH*/))
		      {
           //
		       sID = SkillSet::GoToPoint;
               sParam.GoToPointP.align = true;
               float ballgoaldist = Vector2D<int>::dist(state->ballPos, Vector2D<int>(OPP_GOAL_X, 0));
                 // sprintf(debug,"ballgoaldist = %f\n",ballgoaldist);
                  ////Client::debugClient->SendMessages(debug);
               float offset = 600;
               float factor = 0.00005;
               int targetX = state->ballPos.x + (int)distBotBall  * factor * avgBallVel.x;
               int targetY = state->ballPos.y + (int)distBotBall  * factor * avgBallVel.y;
               int x3 = (targetX * (ballgoaldist + offset) - offset * OPP_GOAL_X) / ballgoaldist;
               int y3 = (targetY * (ballgoaldist + offset)) / ballgoaldist;
               while(!isPointinField(Point2D<int>(x3, y3))) 
               {
                if(!isPointinField(state->ballPos))
                {
                 offset= 0;
                 x3 =  (targetX * (ballgoaldist + offset) - offset * OPP_GOAL_X) / ballgoaldist;
                 y3 =  (targetY * (ballgoaldist + offset)) / ballgoaldist;
                 break;
                }
                offset /= 1.25;
                if(offset <= 1.0)
                  break;
                x3 =  (targetX * (ballgoaldist + offset) - offset * OPP_GOAL_X) / ballgoaldist;
                y3 =  (targetY * (ballgoaldist + offset)) / ballgoaldist;
               }
               offset/=1.25;
               Vector2D<int> offsetpt(x3, y3);
               int dist2 = Vector2D<int>::dist(offsetpt, state->homePos[botID]);
               if(dist2 < 300)
                 hasAchievedOffset = 1;
		       else 
				   if(distBotBall  > 2 * offset)
                     hasAchievedOffset = 0; 

               if(ForwardX(state->ballPos.x) < ForwardX(state->homePos[botID].x)) 
                hasAchievedOffset = 0;
            
               sParam.GoToPointP.x = x3;
               sParam.GoToPointP.y = y3;
               sParam.GoToPointP.finalslope = Vector2D<int>::angle( Vector2D<int>(OPP_GOAL_X, 0),state->ballPos);
               sParam.GoToPointP.increaseSpeed = 0;
               if(hasAchievedOffset)
               {
                 sParam.GoToPointP.x = state->ballPos.x;
                 sParam.GoToPointP.y = state->ballPos.y;
                 sParam.GoToPointP.finalslope = Vector2D<int>::angle( state->ballPos,state->homePos[botID]);

               }
		    	sParam.GoToPointP.align = false;
              if(ForwardX(state->ballPos.x) < ForwardX(state->homePos[botID].x) && Vector2D<int>::dist(state->ballPos,state->homePos[botID]) < 1000) 
		        sParam.GoToPointP.align = true;
              skillSet->executeSkill(SkillSet::GoToPoint, sParam);
			//
			  break ;
			}
		      else
			  {
			  // condition left to write for side ball movement from side of the GoalKeeper	  
			   if(abs(state->ballPos.y) >= OUR_GOAL_MAXY + 2*BOT_RADIUS)
				{
					sID = SkillSet::GoToPoint;
					sParam.GoToPointP.align = false;
					sParam.GoToPointP.finalslope = 0;
			
					sParam.GoToPointP.x =state->ballPos.x	; // ..................changed
					sParam.GoToPointP.y = state->ballPos.y;
			       skillSet->executeSkill(SkillSet::GoToPoint,sParam) ;
				   break ;
			   }
			   else
			   {
			       sID = SkillSet::GoToPoint;
				   sParam.GoToPointP.x = -HALF_FIELD_MAXX + GOAL_DEPTH + 3*BOT_RADIUS;//can change this if cover goal collides
			       sParam.GoToPointP.finalslope = PI/2;
			       if(state->ballPos.y >0)
				     sParam.GoToPointP.y = OUR_GOAL_MAXY + BOT_RADIUS;
			       else
				     sParam.GoToPointP.y = OUR_GOAL_MINY - BOT_RADIUS;
			       skillSet->executeSkill(SkillSet::GoToPoint, sParam);
			      break;
				   
			   }
			  }
 
	      }
	case SPINNING_CCW :
		if(distBotBall > 1.1*BOT_BALL_THRESH)
		{
			iState = DEFENDING;
			break;
		}
	
		sParam.SpinP.radPerSec = MAX_BOT_OMEGA;//for testing it is divided by 10
		skillSet->executeSkill(SkillSet::Spin, sParam);
		break;
	
	case SPINNING_CW :
	
		if(distBotBall > 1.1*BOT_BALL_THRESH)
		{
			iState = DEFENDING;
			break;
		}
		sParam.SpinP.radPerSec = -MAX_BOT_OMEGA;//for testing it is divided by 10
		skillSet->executeSkill(SkillSet::Spin, sParam);
		break;
	
	case ATTACKING :
		cout<< 2;
		if(state->ballPos.x > 0 || (state->ballPos.x < state->homePos[botID].x))//(-HALF_FIELD_MAXX/4))
		{
			iState = DEFENDING;
			break;
		}
		float ang2 = atan(state->ballPos.y/(state->ballPos.x - ForwardX(-HALF_FIELD_MAXX + GOAL_DEPTH) ));
		float ball_ang = atan(state->ballVel.y/state->ballVel.x);
		float deltaT = 0.016;
		sParam.DWGoToPointP.x = state->ballPos.x + deltaT*state->ballVel.x;
		sParam.DWGoToPointP.y = state->ballPos.y + deltaT*state->ballVel.y;
		sParam.DWGoToPointP.finalSlope = ang2;
		//sParam.GoToPointP.align = true;
		skillSet->executeSkill(SkillSet::DWGoToPoint, sParam);
		break;
     }		
	}

	}; 
	}
#endif //TTDWDefender