#include "skillSet.h"
#include "pathPlanners.h"
#include "beliefState.h"
#include "config.h"
#include "logger.h"
#include <math.h>
#include <stdio.h>
#include <time.h>
#include <fstream>
#include <vector>
using namespace std;
namespace Strategy
{
void SkillSet::goToBallStraight(const SParam &param)
	{
		int vl,vr;
		Vector2D<int> dest,spoint,ref;
		vector<obstacle> obs;
		spoint.x = HALF_FIELD_MAXX-DBOX_WIDTH*2;
		spoint.y=0;
		dest.x = dest.y=0;
		ref.x=1;ref.y=0;
		static bool reset = false;
		Vector2D<int> displacement = dest - state->homePos[botID];
		float ang_disp = normalizeAngle(Vector2D<int>::angle(displacement,ref));
		float dist = Vector2D<int>::dist(dest, state->homePos[botID]);   
		float init_angle = fabs(ang_disp - state->homeAngle[botID]);
    comm->addCircle(dest.x,dest.y,BOT_BALL_THRESH/2);
		
		if(dist>BOT_BALL_THRESH)
		{
		if (fabs(ang_disp)<PI/20)
		{
				vl = vr = 70;
				comm->sendCommand(botID,vl,vr);
		}
		else 
			{
					float turnAngleLeft = normalizeAngle(ang_disp); // Angle left to turn

    if(turnAngleLeft>PI/2||turnAngleLeft<-PI/2)
    {
      if(turnAngleLeft>PI/2)
        turnAngleLeft=turnAngleLeft-PI;
      else
        turnAngleLeft=turnAngleLeft+PI;
    }
    float factor = (turnAngleLeft+0.1*(turnAngleLeft))/(PI/2);

    vr = 0.4*MAX_BOT_SPEED*(factor)/(PI/2);

    //vr = -MAX_BOT_SPEED*(turnAngleLeft)/(PI/2);
    vl = -vr;
//    vr = -MAX_BOT_SPEED*turnAngleLeft/(PI/2);
//    vl = -vr;
    //printf("turn angle left: %f\n", turnAngleLeft);
    //printf("State home angle: %f",state->homeAngle[botID]);
    //printf("State Home pos: %d %d", state->homePos[botID].x,state->homePos[botID].y);
    
    if(fabs(turnAngleLeft) > DRIBBLER_BALL_ANGLE_RANGE/2) 
			{
#if FIRA_COMM || FIRASSL_COMM
    comm->sendCommand(botID, vl, vr);        
#else
    comm->sendCommand(botID, vr, vl);
#endif
    }
		else 
		{
      comm->sendCommand(botID, 0, 0);
    }
		  } 
			
			
		}
		else
			comm->sendCommand(botID,0,0);
		}
  /*    FILE *mf;
      mf =  fopen("/home/robo/FIRA_BOT_TEST/Test_gotopoint.txt","a+");
		if(reset==false)
			 SkillSet::_goToPoint(botID,spoint,0,0,0);
		
	//	printf("\n init_angle = %f\n",init_angle);
		  
		if( (Vector2D<int>::dist(spoint, state->homePos[botID]) < BOT_BALL_THRESH && init_angle<0.5) || reset)
			 {
				 reset = true;
			//	printf("\n asdasda \n");
				static clock_t tim=clock();
				static int count =1;
				float vl,vr;
				vl=vr=0;
				static bool reach = false;
				
    
				Vector2D<int> point, nextWP, nextNWP;
				float r = 0, t = 0, dist = 0;
				dist = Vector2D<int>::dist(dest, state->homePos[botID]);  // Distance of next waypoint from the bot
   
#ifdef LOCAL_AVOID
      pathPlanner->plan(state->homePos[botID],
                      state->homeVel[botID],
                      dest,
                      obs,
                      botID,
                      true,
                      state->homeAngle[botID],
                      0,
                      t,
                      r,
                      comm,
                      0);
#else
#error LOCAL_AVOIDANCE should always be defined!
#endif
bool increaseSpeed = true;
  float fTheta = asin(sqrt(fabs(r)));
    fTheta = 1 - fTheta/(PI/2);
    fTheta = pow(fTheta,2.2) ;
    float fDistance = (dist > BOT_POINT_THRESH*3) ? 1 : dist / ((float) BOT_POINT_THRESH *3);
    float fTot = fDistance * fTheta;
    fTot = 0.2 + fTot*(1-0.2);
    float profileFactor = MAX_BOT_SPEED * fTot;
    if(increaseSpeed==1&&r<0.11)
      profileFactor*=2;
//    float profileFactor=(MAX_BOT_SPEED/4.0 + 3/4.0* MAX_BOT_SPEED*(1-fabs(r)));
//    profileFactor*=1.2;
//    profileFactor = MAX_BOT_SPEED;
//    //to limit change
    {
      if(fabs(profileFactor-prevVel)>MAX_BOT_LINEAR_VEL_CHANGE)
      {
        if(profileFactor>prevVel)profileFactor=prevVel+MAX_BOT_LINEAR_VEL_CHANGE;
        else profileFactor=prevVel-MAX_BOT_LINEAR_VEL_CHANGE;
      }
      prevVel=profileFactor;
    }
    if(profileFactor>1.5*MAX_BOT_SPEED)
      profileFactor = 1.5*MAX_BOT_SPEED;
    else if(profileFactor <-1.5*MAX_BOT_SPEED)
      profileFactor = -1.5*MAX_BOT_SPEED;
    prevVel=profileFactor;
    r *= 0.5*profileFactor;
    t *= profileFactor;
  //  printf("Going to Point\n");
//    printf("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!coordinates of bot%d = %d, %d\n",botID,state->homePos[botID].x,state->homePos[botID].y);    
#if FIRA_COMM || FIRASSL_COMM
    comm->sendCommand(botID, (t - r), (t + r));
#else
    comm->sendCommand(botID, (t - r), (t + r));
#endif
		dist = Vector2D<int>::dist(dest, state->homePos[botID]);  
				if(dist < BOT_POINT_THRESH/2 || reach==true )
				{
     
					reach = true;
					vl = vr = 0;
					comm->sendCommand(botID, vl, vr);
					
				}
				if(state->homeVel[botID].x==0 && state->homeVel[botID].y==0 && reach==true)
				{
					if(count==1)
					fprintf(mf,"\n No.       ERROR         ERROR%      			   TIME \n");
					
					float dist = Vector2D<int>::dist(dest, state->homePos[botID]);  
					clock_t sec = clock()-tim;
					fprintf(mf,"\n %d    :    %f      %f   	%f",count,dist,(dist/HALF_FIELD_MAXX)*100,((float)sec)/CLOCKS_PER_SEC );
					tim = clock();
					count++;
					reset = false;
					reach = false;
				} 
		   
			

				}
				
     */ 
		 
		 
			}

