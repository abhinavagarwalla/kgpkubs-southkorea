// This file contains all the skillSet functons
#include <cassert>
#include <string>
#include "skillSet.h"
#include "pathPlanners.h"
#include "beliefState.h"
#include "geometry.hpp"
#include "dataBox.h"
#include "grSimComm.h"
#include "sslComm.h"
#include "logger.h"
using namespace HAL;
using namespace std;
using namespace dlib;
namespace Strategy
{
#if GR_SIM_COMM
Comm* SkillSet::comm = new GrSimComm();
#elif SIM_COMM
Comm* SkillSet::comm = new SimComm();
#elif SSL_COMM
Comm*   SkillSet::comm = new SSLComm();
#elif FIRASSL_COMM
Comm*  SkillSet::comm = new FIRAComm();
#else
# error Macro for Comm class not defined
#endif
string SkillSet::skillsCollection           = "Skills";
string SkillSet::goToPointCollection        = ".GotoPoint";
bool  SkillSet::loadParamsFromFile          = true;
bool  SkillSet::skillParamsLoaded           = false;
float SkillSet::Bot_Point_dis_penalty_l1    = BOT_POINT_THRESH;
float SkillSet::Bot_Point_dis_penalty_l2    = BOT_POINT_THRESH;
float SkillSet::angle_penalty_l1            = DRIBBLER_BALL_ANGLE_RANGE;
float SkillSet::angle_penalty_l2            = DRIBBLER_BALL_ANGLE_RANGE;
float SkillSet::GoToPoint_omegaFactor       = 1.0;
float SkillSet::GoToPoint_profileFactorMult = 1.0;
int   SkillSet::dribble_ball_threshold      = DRIBBLER_BALL_THRESH;
int   SkillSet::bot_ball_threshold          = BOT_BALL_THRESH;
decision_function<radial_basis_kernel<matrix<double, 3, 1> > > SkillSet::gotopoint_profilefac;
decision_function<radial_basis_kernel<matrix<double, 3, 1> > > SkillSet::gotopoint_omegafac;
SkillSet::SkillSet(const BeliefState* state, const int botID) :
  state(state),
  botID(botID),
  prevVel(0)
{
  // Resetting the skill list
  for (int sID = 0; sID < MAX_SKILLS; ++sID)
  {
    skillList[sID] = NULL;
  }
  // Mapping the skill IDs to their corresponding function pointers
  skillList[Spin]           = &SkillSet::spin;
  skillList[Stop]           = &SkillSet::stop;
  skillList[SpinToGoal]     = &SkillSet::spinToGoal;
  skillList[Velocity]       = &SkillSet::velocity;
  skillList[GoToBall]       = &SkillSet::goToBall;
  skillList[GoToPoint]      = &SkillSet::goToPoint;
  skillList[DribbleToPoint] = &SkillSet::dribbleToPoint;
  skillList[TurnToAngle] = &SkillSet::turnToAngle;
  skillList[TurnToPoint] = &SkillSet::turnToPoint;
  skillList[DefendPoint] = &SkillSet::defendPoint;
  skillList[GoalKeeping] = &SkillSet::goalKeeping;
  skillList[GoToPointStraight] = &SkillSet::goToPointStraight;
  skillList[GoToBallStraight] = &SkillSet::goToBallStraight;
  skillList[GoToPointGoalie]      = &SkillSet::goToPointGoalie;
	skillList[ChargeBall] 					= &SkillSet::chargeBall;
  // Initialization check
  for (int sID = 0; sID < MAX_SKILLS; ++sID)
  {
    assert(skillList[sID] != NULL); // Skill enum not mapped to the corresponding skill function
  }
  // Loading Parameters from DataBox
  //errt = new ERRT(0.2f, 0.6f);
#ifdef LOCAL_AVOID
  pathPlanner = new LocalAvoidance();
  pathPlannerSec = new MergeSCurve();
#else
  pathPlanner = new MergeSCurve();
#endif
} // SkillSet
SkillSet::~SkillSet()
{
  delete errt;
  delete pathPlanner;
#ifdef LOCAL_AVOID
  delete pathPlannerSec;
#endif
} // ~SkillSet
/*TODO: Adding definitions of arbit functions, needs to be corrected (see SkillSet.h) */
bool SkillSet::pointyInField(Vector2D<int> final)
{
  if(final.y < -HALF_FIELD_MAXY + BALL_AT_CORNER_THRESH || final.y > HALF_FIELD_MAXY - BALL_AT_CORNER_THRESH)
    return false;
  else return true;
}
//TODO: Try and replace all _ functions with correct implementations
void SkillSet::errorLog(int botid,float vl,float vr, const BeliefState *state)
{
  static float predicted_distance;
  static float predicted_rotation;
  static Point2D<int> prevpoint;
  static float prevorientation;
  static float errordistance;
  static float errororientation;
  /***************by Prasann*********/
  float t=(vl+vr)/2;
  float r=(vl-vr)/2;
  errordistance = (float) fabs(Vector2D<int>::dist(prevpoint,state->homePos[botid])-predicted_distance );
  errororientation = normalizeAngle((state->homeAngle[botid]-prevorientation) - predicted_rotation);
//  printf("/*******error********/ \n distance = %f \n orientation = %f \n/******error*******/\n",errordistance,errororientation);
  prevpoint = state->homePos[botid];
  prevorientation = state->homeAngle[botid];
//  FILE *f = fopen("/home/robo/botplot/compare_dataset/botlog.txt", "a");
  //FILE *f1 = fopen("/home/robo/botplot/compare_dataset/botlogpoint.txt", "a");
//  fprintf(f, "%d %d %d %d %f %f %f %d %d\n", (int)vl, 
//                                             (int)vr, 
//                                             (int)state->homePos[botid].x, 
//                                             (int)state->homePos[botid].y, 
//                                             state->homeAngle[botid], 
//                                             fabs(errordistance),
//                                             errororientation, 
//                                             state->ballPos.x, 
//                                             state->ballPos.y);
  //fprintf(f1,"%f\t%f\n",errordistance,predicted_distance );
//  fclose(f);
  //fclose(f1);
  predicted_distance = t;
  predicted_rotation = r;
  /***************by Prasann*********/
}
void SkillSet::_goToPoint(int botid, Vector2D<int> dpoint, float finalvel, float finalslope, float clearance = CLEARANCE_PATH_PLANNER,bool increaseSpeed)
  {
    comm->addCircle(0, 0, 0);
    comm->addLine(0, 0, 0, 0);
    std::vector<obstacle> obs;
    obstacle obsTemp;
    comm->addLine(state->ballPos.x, state->ballPos.y, state->ballPos.x + state->ballVel.x, state->ballPos.y + state->ballVel.y, 0x000000);
    for (int i = 0; i < HomeTeam::SIZE ; ++i)
    {
      /// Adding Condition to REMOVE all obstacles that are sufficiently CLOSE to BALL
      if( i != botid && Vector2D<int>::distSq(state->homePos[botid], state->homePos[i]) < COLLISION_DIST * COLLISION_DIST && Vector2D<int>::distSq(state->ballPos, state->homePos[i]) > BOT_RADIUS * BOT_RADIUS * 9)
      {
        /// If destination is close to bot, don't add it as obstacle
        if(Vector2D<int>::distSq(dpoint, state->homePos[i]) > BOT_RADIUS * BOT_RADIUS * 1)
        {
          obsTemp.x = state->homePos[i].x;
          obsTemp.y = state->homePos[i].y;
          obsTemp.radius =  BOT_RADIUS;
          obs.push_back(obsTemp);
          comm->addCircle(obs[obs.size() - 1].x, obs[obs.size() - 1].y, BOT_RADIUS * 2);
        }
      }
    }

    for (int i = 0; i < AwayTeam::SIZE; ++i)
    {
      if(Vector2D<int>::distSq(state->homePos[botid], state->awayPos[i]) < COLLISION_DIST * COLLISION_DIST && Vector2D<int>::distSq(state->ballPos, state->awayPos[i]) > BOT_RADIUS * BOT_RADIUS * 9)
      {
        /// If destination is close to bot, don't add it as obstacle
        if(Vector2D<int>::distSq(dpoint, state->homePos[i]) > BOT_RADIUS * BOT_RADIUS * 1)
        {
          obsTemp.x = state->awayPos[i].x;
          obsTemp.y = state->awayPos[i].y;
          obsTemp.radius =  BOT_RADIUS;
          obs.push_back(obsTemp);
          comm->addCircle(obs[obs.size() - 1].x, obs[obs.size() - 1].y, BOT_RADIUS * 2);
        }
      }
    }
    Vector2D<int> point, nextWP, nextNWP;
    float r = 0, t = 0, dist = 0;
    dist = Vector2D<int>::dist(dpoint, state->homePos[botID]);  // Distance of next waypoint from the bot
    
    
    if(dist < BOT_POINT_THRESH )
    {
      float angle = fabs(firaNormalizeAngle(state->homeAngle[botID] - finalslope));
      if(angle > DRIBBLER_BALL_ANGLE_RANGE)
      {
        SParam param;
        param.TurnToAngleP.finalslope=finalslope;
       turnToAngle(param);
        return;
      }
     finalvel=0;
      comm->sendCommand(botID, finalvel, finalvel);
      return;
      // This part of the function is just for safety.
      // The tactic should actually prevent this call.
      // The bot should be aligned properly before this condition is reached.
    }

#ifdef LOCAL_AVOID
    pathPlanner->plan(state->homePos[botID],
                      state->homeVel[botID],
                      dpoint,
                      obs,
                      botID,
                      true,
                      state->homeAngle[botID],
                      finalslope,
                      t,
                      r,
                      comm,
                      clearance);
#else
#error LOCAL_AVOIDANCE should always be defined!
#endif
    float fTheta = asin(sqrt(fabs(r)));
    fTheta = 1 - fTheta/(PI/2);
    fTheta = pow(fTheta,2.2) ;
    float fDistance = (dist > BOT_POINT_THRESH*3) ? 1 : dist / ((float) BOT_POINT_THRESH *3);
    float fTot = fDistance * fTheta;
    fTot = 0.2 + fTot*(1-0.2);
    float profileFactor = MAX_BOT_SPEED * fTot;
    //if(increaseSpeed==1&&r<0.11)
    if(fabs(r)<0.11)
      profileFactor*=2;
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
		//Arpit : Adding polar based code here
		{
			
		}
#if FIRA_COMM || FIRASSL_COMM
    comm->sendCommand(botID, (t - r), (t + r));
#else
    comm->sendCommand(botID, (t - r), (t + r));
#endif
}

void SkillSet::_goToPointLessThresh(int botid, Vector2D<int> dpoint, float finalvel, float finalslope, float clearance = CLEARANCE_PATH_PLANNER,bool increaseSpeed)
{
  comm->addCircle(0, 0, 0);
    comm->addLine(0, 0, 0, 0);
    std::vector<obstacle> obs;
    obstacle obsTemp;
    comm->addLine(state->ballPos.x, state->ballPos.y, state->ballPos.x + state->ballVel.x, state->ballPos.y + state->ballVel.y);
    for (int i = 0; i < HomeTeam::SIZE ; ++i)
    {
      /// Adding Condition to REMOVE all obstacles that are sufficiently CLOSE to BALL
      if( i != botid && Vector2D<int>::distSq(state->homePos[botid], state->homePos[i]) < 0.25*COLLISION_DIST * COLLISION_DIST && Vector2D<int>::distSq(state->ballPos, state->homePos[i]) > BOT_RADIUS * BOT_RADIUS * 9)
      {
        /// If destination is close to bot, don't add it as obstacle
        if(Vector2D<int>::distSq(dpoint, state->homePos[i]) > BOT_RADIUS * BOT_RADIUS * 1)
        {
          obsTemp.x = state->homePos[i].x;
          obsTemp.y = state->homePos[i].y;
          obsTemp.radius =  BOT_RADIUS;
          obs.push_back(obsTemp);
          comm->addCircle(obs[obs.size() - 1].x, obs[obs.size() - 1].y, BOT_RADIUS * 2);
        }
      }
    }

    for (int i = 0; i < AwayTeam::SIZE; ++i)
    {
      if(Vector2D<int>::distSq(state->homePos[botid], state->awayPos[i]) < 0.25*COLLISION_DIST * COLLISION_DIST && Vector2D<int>::distSq(state->ballPos, state->awayPos[i]) > BOT_RADIUS * BOT_RADIUS * 9)
      {
        /// If destination is close to bot, don't add it as obstacle
        if(Vector2D<int>::distSq(dpoint, state->homePos[i]) > BOT_RADIUS * BOT_RADIUS * 1)
        {
          obsTemp.x = state->awayPos[i].x;
          obsTemp.y = state->awayPos[i].y;
          obsTemp.radius =  BOT_RADIUS;
          obs.push_back(obsTemp);
          comm->addCircle(obs[obs.size() - 1].x, obs[obs.size() - 1].y, BOT_RADIUS * 2);
        }
      }
    }
    Vector2D<int> point, nextWP, nextNWP;
    float r = 0, t = 0, dist = 0;
    dist = Vector2D<int>::dist(dpoint, state->homePos[botID]);  // Distance of next waypoint from the bot
    
    
    if(dist < BOT_POINT_THRESH /2 )
    {
      float angle = fabs(firaNormalizeAngle(state->homeAngle[botID] - finalslope));
      if(angle > DRIBBLER_BALL_ANGLE_RANGE)
      {
        SParam param;
        param.TurnToAngleP.finalslope = finalslope;
        turnToAngle(param);
        return;
      }
     
      comm->sendCommand(botID, finalvel, finalvel);
      return;
      // This part of the function is just for safety.
      // The tactic should actually prevent this call.
      // The bot should be aligned properly before this condition is reached.
    }

#ifdef LOCAL_AVOID
    pathPlanner->plan(state->homePos[botID],
                      state->homeVel[botID],
                      dpoint,
                      obs,
                      botID,
                      true,
                      state->homeAngle[botID],
                      finalslope,
                      t,
                      r,
                      comm,
                      clearance);
#else
#error LOCAL_AVOIDANCE should always be defined!
#endif
    float fTheta = asin(sqrt(fabs(r)));
    fTheta = 1 - fTheta/(PI/2);
    fTheta = pow(fTheta,2.2) ;
    float fDistance = (dist > BOT_POINT_THRESH*3) ? 1 : dist / ((float) BOT_POINT_THRESH *3);
    float fTot = fDistance * fTheta;
    fTot = 0.2 + fTot*(1-0.2);
    float profileFactor = MAX_BOT_SPEED * fTot;
    //if(increaseSpeed==1&&r<0.11)
    if(fabs(r)<0.11)
      profileFactor*=2;
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
#if FIRA_COMM || FIRASSL_COMM
    comm->sendCommand(botID, (t - r), (t + r));
#else
    comm->sendCommand(botID, (t - r), (t + r));
#endif
}

void SkillSet::_dribbleToPoint(int botid, Vector2D<int> dpoint, float finalvel, float finalslope, float clearance = 80.0)
{
  float theta   = normalizeAngle(Vector2D<int>::angle(dpoint, state->homePos[botID]));
  float phiStar = finalslope;
  float phi     = (state->homeAngle[botID] < -PI) ? PI : state->homeAngle[botID];
  float dist    = Vector2D<int>::dist(dpoint, state->homePos[botID]);  // Distance of next waypoint from the bot
  float alpha   = normalizeAngle(theta - phiStar);
  float beta;
  //if(fabs(alpha)  < fabs(atan2(clearance, dist)))
  //  printf("ALPHA!\n");
  //else
    //printf("c/d\n");
  beta = (fabs(alpha) < fabs(atan2(clearance, dist))) ? (alpha) : SGN(alpha) * atan2(clearance, dist);
  float thetaD  = normalizeAngle(theta + beta);
  float delta   = normalizeAngle(thetaD - phi);
  float r       = (sin(delta) * sin(delta) * SGN(sin(delta)));
  float t       = (cos(delta) * cos(delta)); //SGN(tan(delta));
  float ballBotAngle = fabs(normalizeAngle(Vector2D<int>::angle(state->ballPos, state->homePos[botID]) - state->homeAngle[botID]));
  if(ballBotAngle > PI / 2)
  {
    t = -t;
    r = -r;
  }
  //printf("delta  = %f\n", ballBotAngle * 180 / PI);
  float profileFactor;
  profileFactor = MAX_BOT_SPEED * ((dist > MIN_DIST_FROM_TARGET) ? (1) : (dist / MIN_DIST_FROM_TARGET));
  //printf("dist= %f\n", dist);
  if(dist < BOT_POINT_THRESH)
  {
#if FIRA_COMM || FIRASSL_COMM
    comm->sendCommand(botID, profileFactor * (t - r), profileFactor * (t + r));
#else
    comm->sendCommand(botID, profileFactor * (t - r), profileFactor * (t + r));
#endif
  }
  else
  {
    comm->sendCommand(botID, 0, 0);
  }
}
void SkillSet::_goToPointStraight(int botid, Vector2D<int> dpoint, float finalvel, float finalslope, float clearance)
{
  float t;
  float phiStar, theta = normalizeAngle(Vector2D<int>::angle(dpoint, state->homePos[botID]));
  phiStar = finalslope;
  float phi = state->homeAngle[botID];
  float dist = Vector2D<int>::dist(dpoint, state->homePos[botID]);  // Distance of next waypoint from the bot
  //  float alpha = firaNormalizeAngle(theta - phiStar);
//    float beta;
//    beta = (fabs(alpha) < fabs(atan2(clearance, dist))) ? (alpha) : SGN(alpha) * atan2(clearance, dist);
  //  float thetaD = normalizeAngle(theta + beta);
  float delta = normalizeAngle(theta - phi);
  float ndelta = firaNormalizeAngle(delta);
  float r = 0.7*(sin(ndelta) * sin(ndelta)) * SGN(tan(ndelta));
  t = (cos(ndelta) * cos(ndelta)) * SGN(cos(delta));
  float profileFactor, velMod;
  velMod = ((state->homeVel[botID].x) * (state->homeVel[botID].x)) + ((state->homeVel[botID].y) * (state->homeVel[botID].y));
  velMod = sqrtf(velMod);
  //profileFactor =  ((dist>MIN_DIST_FROM_TARGET)?(MAX_BOT_SPEED):((MAX_BOT_SPEED-finalvel)*(dist/MIN_DIST_FROM_TARGET)+finalvel));
  //profileFactor = fabs(t) < 0.5 ? MAX_BOT_SPEED * 0.5 : MAX_BOT_SPEED * fabs(t) ;
  profileFactor=(MAX_BOT_SPEED/4.0 + 3/4.0* MAX_BOT_SPEED*(fabs(cos(ndelta))));
  Util::Logger::toStdOut("t: %f,r: %f",t,r);
  if(dist < BOT_POINT_THRESH )
  {
    SParam param;
    param.TurnToAngleP.finalslope = finalslope;
    stop(param);
  }
  else
  {
#if FIRA_COMM || FIRASSL_COMM
    comm->sendCommand(botID, profileFactor * (t - r), profileFactor * (t + r));
#else
    comm->sendCommand(botID, profileFactor * (t - r), profileFactor * (t + r));
#endif
  }
}
void SkillSet::_turnToAngle(float angle, float *vl, float *vr)
{
  //Util::Logger::toStdOut("Bot turning by angle : %f\n",angle);
  float omega = (angle * 2 / PI) * MAX_BOT_OMEGA;
  if(angle < 0)
  {
    //*vl = omega < MIN_BOT_OMEGA ? MIN_BOT_OMEGA*BOT_RADIUS : omega*BOT_RADIUS;
    *vl = 5;
    *vr = -(*vl);
  }
  else
  {
    //*vl = omega < MIN_BOT_OMEGA ? -MIN_BOT_OMEGA*BOT_RADIUS : -omega*BOT_RADIUS;
    *vl = 5;
    *vr = -(*vl);
  }
}
} // namespace Strategy
