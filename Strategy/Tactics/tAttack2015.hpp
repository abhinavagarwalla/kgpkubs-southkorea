#ifndef TTATTACK2015_HPP
#define TTATTACK2015_HPP

#include <list>
#include "comdef.h"
#include "tactic.h"
#include "skillSet.h"
#include "beliefState.h"
#include "logger.h"
#include "config.h"
#include "intersection.hpp"
#define ANGLE_TO_DIST 0
namespace Strategy
{
  class TAttack2015 : public Tactic
  {
    static const int offset = 400;
    // Corner Case: wall, ball, bot in line
    float movementError[10]; // records for previous 10 frames
    float movementErrorSum;
    int movementErrorIndex;
    Point2D<int> prevBotPos;
    float prevBotAngle;
  public:
    TAttack2015(const BeliefState* state, int botID) :
      Tactic(Tactic::Attack2015, state, botID)
    {
      iState = APPROACHING;
      for(int i=0; i<10; i++)
        movementError[i] = 0;
      movementErrorSum  = 0;
      movementErrorIndex = 0;
      prevBotPos = state->homePos[botID];
      prevBotAngle = state->homeAngle[botID];
      hasAchievedOffset = 0;
    } // TAttack

    ~TAttack2015()
    { } // ~TAttack
   enum InternalState
    {
      APPROACHING,
      SPINNING_CCW ,
      SPINNING_CW,
      ATTACKING,
      CLOSE_TO_BALL,
      STUCK
    } iState;
	
    int hasAchievedOffset;
    inline bool isActiveTactic(void) const
    {
      return true;
    }

    int chooseBestBot(std::list<int>& freeBots, const Tactic::Param* tParam, int prevID) const
    {
      int minv = *(freeBots.begin());
	  float angle_difference = firaNormalizeAngle(Vector2D<int>::angle(Vector2D<int>(OPP_GOAL_X, 0), state->ballPos)- state->homeAngle[*(freeBots.begin())]);
      int minwt = Vector2D<int>::dist(state->homePos[*(freeBots.begin())],state->ballPos) + angle_difference * ANGLE_TO_DIST;
	  
	  for (std::list<int>::iterator it = freeBots.begin(); it != freeBots.end(); ++it)
      {
        // TODO make the bot choosing process more sophisticated, the logic below returns the 1st available bot
        float dis_from_ball = Vector2D<int>::dist(state->homePos[*it],state->ballPos);
        float botballangle = normalizeAngle(Vector2D<int>::angle(state->ballPos, state->homePos[*it]));
        //TODO might require normalization
        float botball_orientation_diff = MIN(fabs(botballangle-state->homeAngle[*it]),fabs(botballangle-(state->homeAngle[*it]+PI)));
        float finalOrientationDiff = normalizeAngle(Vector2D<int>::angle(Vector2D<int>(OPP_GOAL_X, 0), state->ballPos)-botballangle);
//		angle_difference =  fabs(firaNormalizeAngle(state->homeAngle[*it]-normalizeAngle(Vector2D<int>::angle(Vector2D<int>(OPP_GOAL_X, 0), state->ballPos))))+ fabs(firaNormalizeAngle((Vector2D<int>::angle(state->homePos[*it],Vector2D<int>(OPP_GOAL_X, 0)))));
      angle_difference = botball_orientation_diff + finalOrientationDiff;
  //float x_diff = ForwardX(state->ballPos.x)-ForwardX(state->homePos.x);
				float weight;
    //printf("%d >>>>>>>>>> %f , %f\n", *it,dis_from_ball,angle_difference);
				weight = dis_from_ball + ANGLE_TO_DIST * angle_difference;
				if(*it == prevID)
					weight -= HYSTERESIS;
        if(weight < minwt)
        {
          minwt = dis_from_ball ;
          minv = *it;
        }
      }
      //Util::Logger::toStdOut("Selected bot %d\n", minv);
	  printf(" :: %d ::",minv);
      //assert(tParam=0);
      return minv;

    } // chooseBestBot
/////////////////////////////////////////

 bool pointxInField(Vector2D<int> final)
 {
      // checks if the point is in the field 
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
//***********
 int opponentProbableGoalkeeper(){
// returns the id of opponent goal keeper 
    int distance=HALF_FIELD_MAXX/2;
    int id=-1;
    for(int i=0; i <5 ; i++){
    if(HALF_FIELD_MAXX-ForwardX(state->awayPos[i].x) < distance){
      if(state->awayPos[i].y < OPP_GOAL_MAXY + 2*BOT_RADIUS && state->awayPos[i].y > OPP_GOAL_MINY -2*BOT_RADIUS){ 
          distance=HALF_FIELD_MAXX-ForwardX(state->awayPos[i].x);
          id=i;
      }
	 }
    } 
  return id;
  }
  
 Vector2D<int> decidedGoalPoint(){
   int id=opponentProbableGoalkeeper();
   Vector2D<int> pointToAttack;
   if(id==-1){
	  pointToAttack.x=HALF_FIELD_MAXX-GOAL_DEPTH;
	  pointToAttack.y=0;
	  return pointToAttack;
   }
   Vector2D<int> pointOfAttack(HALF_FIELD_MAXX-GOAL_DEPTH, 0);
   float errorAngle=0.08726*2;
   float angleGoalMax_y, angleGoalMin_y;
   Vector2D<int> goal_maxy(HALF_FIELD_MAXX-GOAL_DEPTH, OPP_GOAL_MAXY);
   Vector2D<int> goal_miny(HALF_FIELD_MAXX-GOAL_DEPTH, OPP_GOAL_MINY);
   angleGoalMax_y=Vector2D<int>::angle(state->ballPos , goal_maxy)-errorAngle;
   angleGoalMin_y=Vector2D<int>::angle(state->ballPos, goal_miny)+errorAngle;
   if(state->awayPos[id].x > 0.6*OPP_GOAL_MAXY){
        pointOfAttack.y=angleGoalMin_y*(pointOfAttack.x-ForwardX(state->ballPos.x))-state->ballPos.y;
        return pointOfAttack;
   }
   if(state->awayPos[id].x < 0.6*OPP_GOAL_MINY){
      pointOfAttack.y=angleGoalMax_y*(pointOfAttack.x-ForwardX(state->ballPos.x))-state->ballPos.y;
      return pointOfAttack;
   }
   if(state->awayPos[id].y >=0 && state->awayVel[id].y >=0 ){
//	return angleGoalMin_y;
      pointOfAttack.y=angleGoalMin_y*(pointOfAttack.x-ForwardX(state->ballPos.x))-state->ballPos.y;
      return pointOfAttack;
    }
    else if(state->awayPos[id].y <0 && state->awayVel[id].y <0){
    //return angleGoalMax_y;
        pointOfAttack.y=angleGoalMax_y*(pointOfAttack.x-ForwardX(state->ballPos.x))-state->ballPos.y;
        return pointOfAttack;
    }
    return pointOfAttack; 
 }
  
 void shoot(){
    Vector2D<int> targetGoalPoint=decidedGoalPoint();
	Vector2D<int> ballPredictedPos;
	Vector2D<float> ballTransformedVel;
	float factor = 0.00005;
	float botBallAngle=Vector2D<int>::angle(state->ballPos, state->homePos[botID]);
	float botBallDist=Vector2D<int>::dist(state->homePos[botID], state->ballPos);
	float ballGoalPointAngle=Vector2D<int>::angle(ballPredictedPos, targetGoalPoint);
	int ballBotDist = (int)Vector2D<int>::dist(state->homePos[botID],state->ballPos);

    ballPredictedPos.x = state->ballPos.x ;//+ (int)ballBotDist * factor * state->ballVel.x;
    ballPredictedPos.y = state->ballPos.y ;//+ (int)ballBotDist * factor * state->ballVel.y;
    sID=SkillSet::GoToPoint;
    sParam.GoToPointP.x=ballPredictedPos.x;
    sParam.GoToPointP.y=ballPredictedPos.y;
    sParam.GoToPointP.align=true;
    sParam.GoToPointP.finalslope=ballGoalPointAngle;
    sParam.GoToPointP.finalVelocity=MAX_BOT_SPEED;
    skillSet->executeSkill(sID,sParam);
    return;
 }

 int chooseOppReceiver()
 {
	int id=-1;
	for(int i=0;i<5;i++)
	{
		if(state->awayPos[i].x<-HALF_FIELD_MAXX+GOAL_DEPTH+DBOX_WIDTH+5*BOT_RADIUS && abs(state->awayPos[i].y)<OUR_GOAL_MAXY) return i;
	}
	return id;
 }

 bool isBallInDBox()
 {
	if((abs(state->ballPos.y)<OUR_GOAL_MAXY+3*BOT_RADIUS)&&(state->ballPos.x < -HALF_FIELD_MAXX+GOAL_DEPTH+DBOX_WIDTH)) 
	   return true;

		  return false;
 }
/////////////////////////////////////////
   
 void execute(const Param& tParam)
    {
      printf("Attack BotID: %d\n",botID);
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
			
      float dist = Vector2D<int>::dist(state->ballPos, state->homePos[botID]);
      movementError[movementErrorIndex++] = (Vector2D<int>::distSq(prevBotPos, state->homePos[botID])) + (prevBotAngle - state->homeAngle[botID])*(prevBotAngle - state->homeAngle[botID])*50000;
      prevBotPos = state->homePos[botID];
      prevBotAngle = state->homeAngle[botID];
      movementErrorIndex %= 10;
      movementErrorSum = 0;
      for(int i=0; i<10; i++)
        movementErrorSum += movementError[i];
			//printf("movement error = %f, isRotate = %d\n", movementErrorSum, tParam.AttackP.rotateOnError);
      if(movementErrorSum < 500 && tParam.AttackP.rotateOnError)
      {
        sID = SkillSet::Spin;
        sParam.SpinP.radPerSec = MAX_BOT_OMEGA * (state->homePos[botID].y > 0? ForwardX(1): ForwardX(-1));
        skillSet->executeSkill(sID, sParam);
        return;
      }
          
  switch(iState)
  {
        
	case APPROACHING:
	{

	  if(dist<1.1*BOT_BALL_THRESH && state->homePos[botID].x<state->ballPos.x)
	  {
            iState = CLOSE_TO_BALL ;
            break;
	  
	  } 
	  
   if(state->homePos[botID].x>state->ballPos.x && state->homePos[botID].x > -HALF_FIELD_MAXX+GOAL_DEPTH+3*BOT_BALL_THRESH && state->homePos[botID].x < HALF_FIELD_MAXX-GOAL_DEPTH-2*BOT_BALL_THRESH)
   {
	   sID = SkillSet::GoBehindBall ;
	   skillSet->executeSkill(sID, sParam);
	   break ;
   }
	  
		  
      // shoot();
	  //  break;
          sID = SkillSet::GoToPoint;
          sParam.GoToPointP.align = true;
          float ballgoaldist = Vector2D<int>::dist(state->ballPos, Vector2D<int>(OPP_GOAL_X, 0));
          float offset = 600;
         
		  //******************** changed**********************
		  float factorx = 0.00005;
		  if(state->ballVel.x<200  )
			  factorx=0.00005;
		  else if(state->ballVel.x<1000  )
			  factorx=0.00005;
		  else if(state->ballVel.x<1400)
			  factorx=0.00006;
		  else if(state->ballVel.x<2000)
			  factorx=0.00007;
		  else if(state->ballVel.x<3000)
			  factorx=0.00008;
		  else factorx=0.00010;

		   float factory = 0.00005;
		  if( state->ballVel.y<200 )
			  factory=0.00002;
		  else if(state->ballVel.y<1000 )
			  factory=0.00004;
		  else if(state->ballVel.y<1400)
			  factory=0.00006;
		  else if(state->ballVel.y<2000)
			  factory=0.00007;
		  else if(state->ballVel.y<3000)
			  factory=0.00008;
		  else factory=0.0001;
		  //****************************************************
		 /*float factorx,factory;
		  factorx=factory=0.00002;*/

          int ballBotDist = (int)Vector2D<int>::dist(state->homePos[botID],state->ballPos);
          int targetX = state->ballPos.x + (int)ballBotDist * factorx * state->ballVel.x;
          int targetY = state->ballPos.y + (int)ballBotDist * factory * state->ballVel.y;
          int x3 = (targetX * (ballgoaldist + offset) - offset * OPP_GOAL_X) / ballgoaldist;
          int y3 = (targetY * (ballgoaldist + offset)) / ballgoaldist;
          while(!LocalAvoidance::isPointInField(Point2D<int>(x3, y3))) 
          {
                    
            if(!LocalAvoidance::isPointInField(state->ballPos))
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
                  /* Bot is already between ball and offset point */
                  //else if(Vector2D<int>::dist(offsetpt,state->ballPos) < 500 + Vector2D<int>::dist(state->homePos[botID],state->ballPos) + dist2) hasAchievedOffset = 1; 
          else if(dist > 2 * offset)
            hasAchievedOffset = 0; 

          if(ForwardX(state->ballPos.x) < ForwardX(state->homePos[botID].x)) 
            hasAchievedOffset = 0;
            
          sParam.GoToPointP.x = x3;
          sParam.GoToPointP.y = y3;
		  sParam.GoToPointP.finalslope = Vector2D<int>::angle( Vector2D<int>(OPP_GOAL_X, 0),state->ballPos);
          sParam.GoToPointP.increaseSpeed = 0;
          if(hasAchievedOffset)
          {
                          //Client::debugClient->SendMessages("yee i have achived offset!!!!!!!!!!!!\n");
            sParam.GoToPointP.x = state->ballPos.x;
            sParam.GoToPointP.y = state->ballPos.y;
            sParam.GoToPointP.finalslope = Vector2D<int>::angle( state->ballPos,state->homePos[botID]);
            sParam.GoToPointP.increaseSpeed = 1;

          }
		  
			sParam.GoToPointP.align = false;
            if(ForwardX(state->ballPos.x) < ForwardX(state->homePos[botID].x) && Vector2D<int>::dist(state->ballPos,state->homePos[botID]) < 1000) 
		  sParam.GoToPointP.align = true;
   //resricting     
	 /*  if((sParam.GoToPointP.x < -HALF_FIELD_MAXX/2)&&(abs(state->ballPos.y)<0.6*HALF_FIELD_MAXY))

		{ 
			sParam.GoToPointP.x  = ForwardX(-HALF_FIELD_MAXX/2);
			
			if(abs(state->ballPos.y) > HALF_FIELD_MAXY*0.6)
			  sParam.GoToPointP.y = state->ballPos.y - SGN(state->ballPos.y)*2*BOT_RADIUS;
			else
              sParam.GoToPointP.y = state->ballPos.y + SGN(state->ballPos.y)*2*BOT_RADIUS;

	    }
    */

	   

	   /////////////////////////////////////////////////////////////
// change the condition of postion of X ,Y when the ball is in D-Box
	  if(isBallInDBox()==true)
	  {
		  sParam.GoToPointP.x =  -HALF_FIELD_MAXX+GOAL_DEPTH+DBOX_WIDTH+BOT_RADIUS ;
		  sParam.GoToPointP.y =   SGN(state->ballPos.y)*(OUR_GOAL_MAXY+BOT_RADIUS);
		  if(Vector2D<int>::dist(state->homePos[botID],state->homePos[0])>2*BOT_BALL_THRESH)
		  { int id=chooseOppReceiver();
		  if(state->ballPos.x<-HALF_FIELD_MAXX+GOAL_DEPTH+2*BOT_RADIUS && state->awayPos[id].x>-HALF_FIELD_MAXX+GOAL_DEPTH+BOT_RADIUS)
		  {
			  if(id!=-1) 
			  {
				  sParam.GoToPointP.x = state->awayPos[id].x-1.2*BOT_RADIUS ;
				  sParam.GoToPointP.y= state->awayPos[id].y;
			  }
		  }
		 }

	  }
      if (state->homePos[0].x + BOT_RADIUS > state->homePos[botID].x)		// will not disturb GoalKeeper from the back: By KD
	  {
	      if (state->ballPos.y > 0)
           {
			  sID = SkillSet::GoToPoint;
			  if (Vector2D<int>::dist(state->homePos[0], state->homePos[botID]) <= 3*BOT_RADIUS)
			  {
				sParam.GoToPointP.align = false;
				sParam.GoToPointP.finalslope = PI/2;
				sParam.GoToPointP.x = state->homePos[botID].x;
				sParam.GoToPointP.y = state->homePos[botID].y - 2*BOT_RADIUS;
			  }
			  else
				{
				sParam.GoToPointP.align = false;
				sParam.GoToPointP.finalslope = 0;
				sParam.GoToPointP.x = state->homePos[botID].x + 2*BOT_RADIUS;
				sParam.GoToPointP.y = state->homePos[botID].y;
				}
			}
			else
			{
				sID = SkillSet::GoToPoint;
				if (Vector2D<int>::dist(state->homePos[0], state->homePos[botID]) <= 3.5*BOT_RADIUS)
				{
				    sParam.GoToPointP.align = false;
					sParam.GoToPointP.finalslope = PI/2;
					sParam.GoToPointP.x = state->homePos[botID].x;
					sParam.GoToPointP.y = state->homePos[botID].y + 2*BOT_RADIUS;
				}
				else
				{
					sParam.GoToPointP.align = false;
					sParam.GoToPointP.finalslope = 0;
					sParam.GoToPointP.x = state->homePos[botID].x + 2*BOT_RADIUS;
					sParam.GoToPointP.y = state->homePos[botID].y;
				}
			 }
				
		}					//Totally working: Approved. Now it works universally
    
	    if(state->ballPos.x>HALF_FIELD_MAXX-GOAL_DEPTH-2*BOT_RADIUS && state->homePos[botID].x>HALF_FIELD_MAXX-GOAL_DEPTH-2*BOT_RADIUS)
		{
		  if(state->ballPos.y>OPP_GOAL_MAXY && state->homePos[botID].y<state->ballPos.y)
		  {
			 sParam.GoToPointP.x=HALF_FIELD_MAXX-GOAL_DEPTH-4*BOT_RADIUS;
			 sParam.GoToPointP.y=state->homePos[botID].y;
		  }
		  if(state->ballPos.y<OPP_GOAL_MINY && state->homePos[botID].y>state->ballPos.y)
		  {
			sParam.GoToPointP.x=HALF_FIELD_MAXX-GOAL_DEPTH-4*BOT_RADIUS;
			sParam.GoToPointP.y=state->homePos[botID].y;
		  }
		}
      
	  skillSet->executeSkill(sID, sParam);
	  break;
    }

	case SPINNING_CW:
	{  
	   if(dist>BOT_BALL_THRESH)
	   {
		 iState = APPROACHING;
		 return;
	   }
		 // shoot();
		 // break;
	   sID = SkillSet::Spin;
	   if(FIELD_IS_INVERTED == false)
		 sParam.SpinP.radPerSec = (MAX_BOT_OMEGA);
	   else
		 sParam.SpinP.radPerSec = -(MAX_BOT_OMEGA);
	   skillSet->executeSkill(sID, sParam);
	  break;    
     }
	case SPINNING_CCW:
	{
	  if(dist>BOT_BALL_THRESH)
	  {
		iState = APPROACHING;
		return;
	  }
         // shoot();
		 // break;
	  sID = SkillSet::Spin;
	  if(FIELD_IS_INVERTED == false)
		sParam.SpinP.radPerSec = -(MAX_BOT_OMEGA);
	  else
		sParam.SpinP.radPerSec = (MAX_BOT_OMEGA);
             
	  skillSet->executeSkill(sID, sParam);
	  break;
	}

	case CLOSE_TO_BALL:
	{
	 if(abs(state->homePos[botID].y)<HALF_FIELD_MAXY-2*BOT_RADIUS && fabs((float)normalizeAngle(state->homeAngle[botID] - atan2(state->homePos[botID].y - state->ballPos.y, state->homePos[botID].x - state->ballPos.x))) < PI / 2 + PI / 20 && fabs((float)normalizeAngle(state->homeAngle[botID] - atan2(state->homePos[botID].y - state->ballPos.y, state->homePos[botID].x - state->ballPos.x)))  > PI / 2 - PI / 20 )
	 {
		if(state->ballPos.y > 0)
		  iState = FIELD_IS_INVERTED? SPINNING_CCW : SPINNING_CW;        //opposite of false
		else
		  iState = FIELD_IS_INVERTED? SPINNING_CW : SPINNING_CCW;        //opposite of flase
		break;
	 }
          /* Ball is with bot. So go to goal */
	        sID = SkillSet::GoToPoint;
	        int desty = 0;
			Vector2D<int> GoalMidPoint  (OPP_GOAL_X,0);
			Vector2D<int> GoalLeftPoint (OPP_GOAL_X,OPP_GOAL_MINY);
			Vector2D<int> GoalRightPoint(OPP_GOAL_X,OPP_GOAL_MAXY);
			float angleofBot = state->homeAngle[botID];
			if(angleofBot > (3 * PI)/4) angleofBot  = (3 * PI)/4;
			if(angleofBot < -(3 * PI)/4) angleofBot = -(3 * PI)/4;
			float R =  (OPP_GOAL_X - state->homePos[botID].x) / ( cos(angleofBot) );
			float destY = state->homePos[botID].y + R * sin(angleofBot);
			if(destY < OPP_GOAL_MINY + 200) destY = OPP_GOAL_MINY + 200;
			if(destY > OPP_GOAL_MAXY - 200) destY = OPP_GOAL_MAXY - 200;
			/*float angleWithGoal  =  Vector2D<int>::angle(state->homePos[botID],GoalMidPoint);
			float leftGoalAngle  =  Vector2D<int>::angle(GoalLeftPoint,state->homePos[botID]);
			float rightGoalAngle =  Vector2D<int>::angle(state->homePos[botID],GoalRightPoint);
            */
			sParam.GoToPointP.align = false;
            sParam.GoToPointP.x = OPP_GOAL_X;
            sParam.GoToPointP.y = destY;
			if(abs(state->ballPos.y)>HALF_FIELD_MAXY-2*BOT_RADIUS && abs(state->homePos[botID].y)>HALF_FIELD_MAXY-1.7 && botID==state->ourBotNearestToBall)
			{
				sParam.GoToPointP.finalslope = Vector2D<int>::angle(state->ballPos,state->homePos[botID]);
			}
			else if(abs(state->ballPos.y)>HALF_FIELD_MAXY-2*BOT_RADIUS && abs(state->homePos[botID].y)>HALF_FIELD_MAXY-1.7*BOT_RADIUS && botID!=state->ourBotNearestToBall)
			{
				sParam.GoToPointP.finalslope = Vector2D<int>::angle(state->homePos[state->oppBotNearestToBall],state->homePos[botID]);
			}
			else
			{
				sParam.GoToPointP.finalslope = Vector2D<int>::angle( Vector2D<int>(OPP_GOAL_X, 0),state->ballPos);
			}
            sParam.GoToPointP.increaseSpeed = 1;
            skillSet->executeSkill(sID, sParam);
            tState = RUNNING;
            iState = CLOSE_TO_BALL; //ATTACKING;
            if(dist > 1.1*BOT_BALL_THRESH)
            {
              iState = APPROACHING ;
			
            }
          break ;
	}
				
      }       
    }  
      
  }; // class TAttack
} // namespace Strategy


#endif // TTCharge_HPP