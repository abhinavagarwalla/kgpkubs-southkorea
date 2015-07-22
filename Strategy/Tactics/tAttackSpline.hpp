#ifndef TTATTACKSPLINE_HPP
#define TTATTACKSPLINE_HPP

#include <list>
#include "comdef.h"
#include "tactic.h"
#include "skillSet.h"
#include "beliefState.h"
#include "logger.h"
#include "config.h"
#include "intersection.hpp"
#include <fstream>
#define ANGLE_TO_DIST 0

namespace Strategy
{
  class TAttackSpline : public Tactic
  {
    static const int offset = 400;
    // Corner Case: wall, ball, bot in line
    float movementError[10]; // records for previous 10 frames
    float movementErrorSum;
    int movementErrorIndex;
    Point2D<int> prevBotPos;
    float prevBotAngle;
	int splin;
	int sCount;
  public:
    TAttackSpline(const BeliefState* state, int botID) :
      Tactic(Tactic::AttackSpline, state, botID)
    {
		sCount = 0 ;
	  splin = 0;
      iState = APPROACHING;
      for(int i=0; i<10; i++)
        movementError[i] = 0;
      movementErrorSum  = 0;
      movementErrorIndex = 0;
      prevBotPos = state->homePos[botID];
      prevBotAngle = state->homeAngle[botID];
      hasAchievedOffset = 0;
    } // TAttack

    ~TAttackSpline()
    { } // ~TAttack
   enum InternalState
    {
      APPROACHING,
      SPINNING_CCW ,
      SPINNING_CW,
      ATTACKING,
      CLOSE_TO_BALL,
      STUCK,
	  OLD_ATTACK,
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

    ballPredictedPos.x = state->ballPos.x ;//+ (int)ballBotDist * factor * avgBallVel.x;
    ballPredictedPos.y = state->ballPos.y ;//+ (int)ballBotDist * factor * avgBallVel.y;
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
	if((abs(state->ballPos.y)<OUR_GOAL_MAXY+ 5*BOT_RADIUS)&&(state->ballPos.x < -HALF_FIELD_MAXX+GOAL_DEPTH+DBOX_WIDTH)) 
	   return true;

		  return false;
 }
/////////////////////////////////////////
   
 void execute(const Param& tParam)
    { 
		
      printf("Attack BotID: %d\n",botID);
      static Vector2D<float> lastVel[10];
			static int index = 0;
			static bool isfirst = true ;
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
			if(isfirst)
			{  avgBallVel.x = 0.0 ; 
			   avgBallVel.y = 0.0 ;   
			}
			avgBallVel.x /= 10.0;
			avgBallVel.y /= 10.0;
			if(state->ballVel.x == 0)
			   avgBallVel.x = 0.0 ;
			if(state->ballVel.y == 0)
			   avgBallVel.y = 0.0 ;
			   
			if(isfirst&&(index==9))
				isfirst = false ;
//		if(sCount++ < 15){
//			sID = SkillSet::Stop;
//			skillSet->executeSkill(sID, sParam);	
//			return ;
//		}
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
        sParam.SpinP.radPerSec = 0.5*MAX_BOT_OMEGA * (state->homePos[botID].y > 0? ForwardX(1): ForwardX(-1));
        skillSet->executeSkill(sID, sParam);
        return;
      }
	  
      ofstream outfile;
      outfile.open("/home/robocup/vel_log.txt", ios::app );
	  outfile<<avgBallVel.x<<" "<<avgBallVel.y<<"    ::   "<<state->ballVel.x<<" "<<state->ballVel.y<<endl;
	  outfile.close() ;
	 
	cout<<"Ball Velocity "<<avgBallVel.x<<" "<<avgBallVel.y<<std::endl ;
  switch(iState)
  {
	case OLD_ATTACK:
	{
		if(dist<1.1*BOT_BALL_THRESH && state->homePos[botID].x<state->ballPos.x )
		{
            iState = CLOSE_TO_BALL ;
            break;
		} 
	}
	case APPROACHING:
	{ 
	  if(dist<1.1*BOT_BALL_THRESH && state->homePos[botID].x<state->ballPos.x )
	  {
            iState = CLOSE_TO_BALL ;
			splin = 0 ;
            break;
	  
	  }
	  if (dist < 1000) {
			iState = OLD_ATTACK;
			splin = 0 ;
            break;
		}
	  cout<<"APPROACHING"<<endl ; 
	  if(isBallInDBox()==true)
	  {
		  sParam.GoToPointP.x =  -HALF_FIELD_MAXX + GOAL_DEPTH + DBOX_WIDTH+BOT_RADIUS ;
		  sParam.GoToPointP.y =   SGN(state->ballPos.y)*(OUR_GOAL_MAXY+BOT_RADIUS);
		  if(Vector2D<int>::dist(state->homePos[botID],state->homePos[0])>2*BOT_BALL_THRESH)
		  { int id=chooseOppReceiver();
		  // required only in the game 
		  /*
		  if(state->ballPos.x<-HALF_FIELD_MAXX+GOAL_DEPTH+2*BOT_RADIUS && state->awayPos[id].x>-HALF_FIELD_MAXX+GOAL_DEPTH+BOT_RADIUS)
		  {
			  if(id!=-1) 
			  {
				  sParam.GoToPointP.x = state->awayPos[id].x-1.2*BOT_RADIUS ;
				  sParam.GoToPointP.y= state->awayPos[id].y;
			  }
		  }
		  */		
		}
       splin = 0 ; 	  
	  skillSet->executeSkill(sID, sParam);
	  break;
	  }
	  
	  // write the code for local avoidance also :: using CP in spline 
	      static Vector2D<int> lastSplinePoint(state->ballPos.x , state->ballPos.y) ;
          cout << "spline here :: " << splin << std::endl;
		  sID = SkillSet::SplineInterceptBall;
		//  cout << "here" << endl;
		  sParam.SplineInterceptBallP.vl = 0;
		  sParam.SplineInterceptBallP.vr = 0;
		  sParam.SplineInterceptBallP.velGiven = 1;
		  sParam.SplineInterceptBallP.ballVelX = avgBallVel.x;
		  sParam.SplineInterceptBallP.ballVelY = avgBallVel.y;
		 // cout << "here" << endl;
		  if(splin == 0){
				sParam.SplineInterceptBallP.initTraj = 1;
				lastSplinePoint.x = state->ballPos.x ; lastSplinePoint.y = state->ballPos.y ;
		  }		
		  else{
			  sParam.SplineInterceptBallP.initTraj = 0;
		  }
		//  cout << "here " << endl;
		  skillSet->executeSkill(sID, sParam);
		  splin  = 1;
		  if(Vector2D<int>::dist(lastSplinePoint , state->ballPos) > 4*BOT_RADIUS)
			{  splin  = 0 ;
		       cout<<"Creating new spline at point :: "<<lastSplinePoint.x<<" "<<lastSplinePoint.y<<std::endl;
			   
 			}    
				break;
    }

	case SPINNING_CW:
	{ 
	   cout<<"SPINNING_CW"<<endl;
	   if(dist>2*BOT_BALL_THRESH)
	   {
		 if (dist < 1000) {
		   iState = OLD_ATTACK;
			return;
		}
		iState = APPROACHING;
		return;
	   }
		 // shoot();
		 // break;
	   sID = SkillSet::Spin;
	   if(FIELD_IS_INVERTED == false)
		 sParam.SpinP.radPerSec = (0.5*MAX_BOT_OMEGA);
	   else
		 sParam.SpinP.radPerSec = -(0.5*MAX_BOT_OMEGA);
	   skillSet->executeSkill(sID, sParam);
	  break;    
     }
	case SPINNING_CCW:
	{
	  cout<<"SPINNING_CCW"<<endl;
	  if(dist>2*BOT_BALL_THRESH)
	  {
		if (dist < 1000) {
		   iState = OLD_ATTACK;
			return;
	   }  
		iState = APPROACHING;
		return;
	  }
         // shoot();
		 // break;
	  sID = SkillSet::Spin;
	  if(FIELD_IS_INVERTED == false)
		sParam.SpinP.radPerSec = -(0.5*MAX_BOT_OMEGA);
	  else
		sParam.SpinP.radPerSec = (0.5*MAX_BOT_OMEGA);
             
	  skillSet->executeSkill(sID, sParam);
	  break;
	}

	case CLOSE_TO_BALL:
	{
		cout<<"CLOSE_TO_BALL"<<endl;
           /*
		   if(fabs(normalizeAngle(state->homeAngle[botID] - atan2(state->homePos[botID].y - state->ballPos.y, state->homePos[botID].x - state->ballPos.x))) < PI / 2 + PI / 9 && fabs(normalizeAngle(state->homeAngle[botID] - atan2(state->homePos[botID].y - state->ballPos.y, state->homePos[botID].x - state->ballPos.x)))  > PI / 2 - PI / 9)
           {
              if(state->ballPos.y > 0)
                iState = FIELD_IS_INVERTED? SPINNING_CCW : SPINNING_CW;
			  else
				iState = FIELD_IS_INVERTED? SPINNING_CW : SPINNING_CCW;
              break ;
		   } 
		   */
		   if(dist > 2*BOT_BALL_THRESH)
		   {
			   if (dist < 1000) {
					iState = OLD_ATTACK;
					break;
				}
              iState = APPROACHING ;
			   break ;
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
			
			// **************  YAHAN PE SPLINE ******************************************************* 
			
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
           
          break ;
	}
				
      }       
    }  
      
  }; // class TAttack
} // namespace Strategy


#endif // TTCharge_HPP