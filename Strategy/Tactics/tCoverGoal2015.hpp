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
    
	// choose best vbot to be changed
	
				//Commented. Test remaining
	int chooseBestBot(std::list<int>& freeBots, const Tactic::Param* tParam, int prevID) const
    {
      int minv = *(freeBots.begin());
      int mindis = 1000000000;
		int factor = 0.2;
	  int destX = ForwardX(-(HALF_FIELD_MAXX-GOAL_DEPTH)*0.80);
      for (std::list<int>::iterator it = freeBots.begin(); it != freeBots.end(); ++it)
      {
		Point2D<int> our_goal(-HALF_FIELD_MAXX + GOAL_DEPTH, 0) ;
        float dist = Vector2D<int>::dist(state->homePos[*it], our_goal) ;
		float perpend_dist = ForwardX(state->homePos[*it].x - ForwardX(-HALF_FIELD_MAXX));
        if(dist + factor * perpend_dist < mindis)
        {
          mindis =  dist + factor * perpend_dist < mindis;
          minv = *it;
        }
      }
	  return minv;
	}
	  /*
    int chooseBestBot(std::list<int>& freeBots, const Tactic::Param* tParam, int prevID) const
    {
      int minv = *(freeBots.begin());
      int mindis = 1000000000;
	  int destX = ForwardX(-(HALF_FIELD_MAXX-GOAL_DEPTH)*0.80);
      for (std::list<int>::iterator it = freeBots.begin(); it != freeBots.end(); ++it)
      {

        float perpend_dist = abs(ForwardX( state->homePos[*it].x - destX ));//state->home_goalpoints[2] is center of our goal
        if(perpend_dist < mindis)
        {
          mindis =  perpend_dist;
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

   int ourGoalieID()
   {
	   
	 Vector2D<int> goalPos(-ForwardX(OUR_GOAL_X),0) ;
	 int id = 0 ;
	 int minDis = HALF_FIELD_MAXX ;
	 for(int i = 0 ; i < 5 ; i++)
	 {
	   if( i != botID)	 
	   {
	      if(Vector2D<int>::dist(goalPos , state->homePos[i]) < minDis)
		  {
		      minDis = Vector2D<int>::dist(goalPos , state->homePos[i]) ;
			  id = i ;
		  }
	   }
	 } 
	 return id ;
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
   void execute(const Param& tParam)
   {
		cout<< "\n kj\n \n"<<iState<<"\n";
	float distBotBall = Vector2D<int>::dist(state->ballPos,state->homePos[botID]);
	int oppInRegion[4]={0},minRegion=0,targetX = INT_MAX,targetY =-SGN(state->ballPos.y)*BOT_RADIUS;
	float ang1 = state->ballVel.x == 0? PI/2 :atan(state->ballVel.y/state->ballVel.x);
	float goalAngR,goalAngL,goalieAngR,goalieAngL,divisor,goalAngC;
	int yL,yR,goalieyR,goalieyL,yC;
	int predictPosX = ForwardX(-(HALF_FIELD_MAXX-GOAL_DEPTH)*0.80);
    Vector2D<int> dest;
	dest.x = ForwardX(-(HALF_FIELD_MAXX-GOAL_DEPTH)*0.80);
    int oppBallDist = Vector2D<int>::dist(state->ballPos , state->awayPos[state->oppBotNearestToBall]) ;
	divisor = state->ballPos.x - ForwardX(-HALF_FIELD_MAXX + GOAL_DEPTH);
	goalAngL = divisor == 0 ?  PI/2 : atan((state->ballPos.y - OUR_GOAL_MAXY)/(divisor));
	goalAngR = divisor == 0 ?  PI/2 : atan((state->ballPos.y - OUR_GOAL_MINY)/(divisor));
	goalAngC = divisor == 0 ?  PI/2 : atan((state->ballPos.y)/(divisor));
	yL = state->ballPos.y - (state->ballPos.x - dest.x)*tan(goalAngL);
	yR = state->ballPos.y - (state->ballPos.x -dest.x)*tan(goalAngR);
	yC = state->ballPos.y - (state->ballPos.x -dest.x)*tan(goalAngC);
	if(yL > OUR_GOAL_MAXY - BOT_RADIUS)
		yL = OUR_GOAL_MAXY - BOT_RADIUS;
	else if(yR < OUR_GOAL_MINY + BOT_RADIUS)
		yR = OUR_GOAL_MINY + BOT_RADIUS;
	

	//for goalie
	// change the id for teh goalie
	int goalieID = ourGoalieID() ;
	divisor = state->ballPos.x - state->homePos[goalieID].x;  
	goalieAngR = divisor ==0? PI/2 : atan((state->ballPos.y - (state->homePos[goalieID].y -BOT_RADIUS))/(divisor));
	goalieAngL = divisor ==0? PI/2 : atan((state->ballPos.y - (state->homePos[goalieID].y +BOT_RADIUS))/divisor);
	goalieyR = state->ballPos.y - (state->ballPos.x - dest.x)*tan(goalieAngR);
	goalieyL = state->ballPos.y - (state->ballPos.x - dest.x)*tan(goalieAngL);
	

	if(OUR_GOAL_MAXY-(state->homePos[goalieID].y +BOT_RADIUS) < 1.5*BOT_RADIUS)
		yL = goalieyR;
	else if(goalieyR - (state->homePos[goalieID].y -BOT_RADIUS) <1.5*BOT_RADIUS)
		yR = goalieyL;
	
	int predictPosY = state->ballPos.y - (state->ballPos.x - predictPosX)*tan(ang1) + SGN(state->ballVel.y)*BOT_RADIUS;
	int goaliePredictPosY = state->ballPos.y - (state->ballPos.x - (-HALF_FIELD_MAXX+GOAL_DEPTH))*tan(ang1);

    oppBallDist = HALF_FIELD_MAXX ;   // TO BE CHANGED AFTER TETING

	// add the info of away bots 
	
	cout << "here 1 " << predictPosX << endl;
	switch (iState)
	{
	case DEFENDING:
		cout << "here " << iState<< endl;
	 if(state->ballPos.x < state->homePos[botID].x)
	 {
	     iState = BLOCKING ;
		 break ;
	 }
	 if( distBotBall < 2*BOT_BALL_THRESH && ( oppBallDist > 4*BOT_BALL_THRESH))
   	 {
		iState = CLEARING;
		cout << "here 2 " << predictPosX << endl;
		break;
	 }
	 if( state->ballPos.x > state->homePos[botID].x)
	 {
		if(state->ballVel.x >=0)
		{
			dest.y = state->ballPos.y;
			if(dest.y>OUR_GOAL_MAXY)
				dest.y = OUR_GOAL_MAXY;
			else if(dest.y < OUR_GOAL_MINY)
				dest.y = OUR_GOAL_MINY;
		}
		else if( goaliePredictPosY > OUR_GOAL_MAXY - BOT_RADIUS)
		{
			dest.y = yC;//OUR_GOAL_MAXY - 3*BOT_RADIUS
		}
		else if(goaliePredictPosY < OUR_GOAL_MINY + BOT_RADIUS)
		{
			dest.y = yC;//OUR_GOAL_MINY + 3*BOT_RADIUS
		}
		else if(predictPosY > yR && predictPosY < yL)
		{
			dest.y = predictPosY;
		}
		else if(predictPosY > yL)
		{
			dest.y = yL - BOT_RADIUS;
		}
		else
		{
			dest.y = yR + BOT_RADIUS;
		}
	}
	sParam.GoToPointP.x = dest.x;
	sParam.GoToPointP.y = dest.y;
	sParam.GoToPointP.finalslope = -PI/2;
	cout<<dest.x <<" "<<dest.y<<std::endl;
	skillSet->executeSkill(SkillSet::GoToPoint, sParam);
	break;
				
	case CLEARING :
		
		if(state->ballPos.x < state->homePos[botID].x)
	    {
	     iState = BLOCKING ;
		 break ;
	    }
		
		if( distBotBall > 2*BOT_BALL_THRESH)
		{
			iState = DEFENDING;
			break;
		}
		/*else if( 0)//fabs(normalizeAngle(state->homeAngle[botID] - atan2(state->homePos[botID].y - state->ballPos.y, state->homePos[botID].x - state->ballPos.x))) < PI /4  && state->ballPos.x >= state->homePos[botID].x)
			{
				if(minRegion ==1)
					sParam.DWGoToPointP.y = -HALF_FIELD_MAXY + HALF_FIELD_MAXY/4;
				else if(minRegion ==2)
					sParam.DWGoToPointP.y = -HALF_FIELD_MAXY/2 + HALF_FIELD_MAXY/4;
				else if(minRegion ==3)
					sParam.DWGoToPointP.y =  HALF_FIELD_MAXY/4;
				else if(minRegion ==4)
					sParam.DWGoToPointP.y = HALF_FIELD_MAXY/2 + HALF_FIELD_MAXY/4;
				sParam.DWGoToPointP.x = state->ballPos.x;
				sParam.DWGoToPointP.y = state->ballPos.y;
				sParam.DWGoToPointP.finalSlope = PI/2;//shouldn't it be 0
				skillSet->executeSkill(SkillSet::DWGoToPoint, sParam);
				break;
			}*/
        //if(fabs(normalizeAngle(state->homeAngle[botID] - atan2(state->homePos[botID].y - state->ballPos.y, state->homePos[botID].x - state->ballPos.x))) < PI / 2 + PI / 10 && fabs(normalizeAngle(state->homeAngle[botID] - atan2(state->homePos[botID].y - state->ballPos.y, state->homePos[botID].x - state->ballPos.x)))  > PI / 2 - PI / 10)
                sID = SkillSet::Spin ;
				if(state->ballPos.y > state->homePos[botID].y)
					sParam.SpinP.radPerSec = -MAX_BOT_OMEGA ;
				else
					sParam.SpinP.radPerSec = MAX_BOT_OMEGA ;
				skillSet->executeSkill(sID, sParam);
				break;		
		
				

	 case BLOCKING :
	 //  write once the opponent is taken into acccount
		if( state->ballPos.x > -ForwardX((HALF_FIELD_MAXX-GOAL_DEPTH)*0.80) )
		{
			iState = DEFENDING;
			break;
		} 
		int oppID = nearestOppBot(dest.x) ;
		
		// get the opponent id from the function
		/*else if(abs(predictPosY) < OUR_GOAL_MAXY )
		{
			sParam.DWGoToPointP.y = predictPosY ;//+SGN(state->ballVel.y)*3*BOT_RADIUS ;
			//if(abs(sParam.DWGoToPointP.y) > HALF_FIELD_MAXY)
				//sParam.DWGoToPointP.y = SGN(sParam.DWGoToPointP.y)*HALF_FIELD_MAXY - SGN(sParam.DWGoToPointP.y)*BOT_RADIUS;
			sParam.DWGoToPointP.finalSlope = PI/2;
			skillSet->executeSkill(SkillSet::DWGoToPoint, sParam);
			break;
		}*/
		if(abs(state->awayPos[oppID].y) > OUR_GOAL_MAXY)
			sParam.GoToPointP.y = SGN(state->awayPos[oppID].y)*OUR_GOAL_MAXY;
		else
			sParam.GoToPointP.y = state->awayPos[oppID].y ;
			sParam.GoToPointP.finalslope = -PI/2;
			sParam.GoToPointP.x = dest.x;
			skillSet->executeSkill(SkillSet::GoToPoint, sParam);
			break;
		
    
	}
   }

}; // class TDefense
} // namespace Strategy

#endif // TTCharge_HPP