#include "skillSet.h"
#include "beliefState.h"
#include "logger.h"
#include "config.h"
#include "pose.h"
#include <iostream>
#include "ballinterception.cpp"

#define PREDICTION_PACKET_DELAY 4
using namespace Util;
static bool direction2 = true;
static bool lastDirection2 = true;

namespace Strategy
{
  void SkillSet::_splineInterceptBallTrack(int botid, Pose start, Vector2D<float> ballPos, Vector2D<float> ballVel, Vector2D<float> botVel, float final_vl, float final_vr){
    
  //  cout << "in this" << endl;
    interceptCounter++;
    
    if(!algoController){
      if(traj)
        algoController = new ControllerWrapper(traj, state->homeVlVr[botID].x, state->homeVlVr[botID].y, PREDICTION_PACKET_DELAY);
    }
    
    if(!direction2)
      start.setTheta(normalizeAngle(start.theta() - PI));
    
    int vl,vr;
    Pose dummy(0,0,0);
    algoController->genControls(start, dummy, vl, vr, 0);
    predictedPoseQ.push_back(algoController->getPredictedPose(start));
    if(predictedPoseQ.size() > PREDICTION_PACKET_DELAY){
        predictedPoseQ.pop_front();
    }
    assert(vl <= 150 && vl >= -150);
    assert(vr <= 150 && vr >= -150);
    if (direction2)
      comm->sendCommand(botid, vl/2, vr/2); //maybe add mutex
    else {
      //cout << "ulta chaloooo" << endl;
      int vl1 = (-1)*vr;
      int vr1 = (-1)*vl;
      comm->sendCommand(botid, vl1/2, vr1/2);
    }
  }

  void SkillSet::_splineInterceptBallInitTraj(int botid, Pose start, Vector2D<float> ballPos, Vector2D<float> ballVel, Vector2D<float> botVel, float final_vl, float final_vr, int flag){

    float dist = Vector2D<int>::dist(state->homePos[botID], state->ballPos);  
    interceptCounter = 0; 
    if(dist < 50){
      comm->sendCommand(botID, 0, 0);
      return;
    }
    Pose start2(state->homePos[botID].x, state->homePos[botID].y, normalizeAngle(state->homeAngle[botID] - PI));
    Vector2D<int> GoalPoint(ForwardX(HALF_FIELD_MAXX), 0);
    direction2 = _isFrontDirected(start, Pose(state->ballPos.x, state->ballPos.y , Vector2D<int>::angle(state->ballPos, GoalPoint)), state->homeVlVr[botID].x, state->homeVlVr[botID].y);
    interceptCounter = 0; 
    
    Vector2D<float> delayedVel(botVel.x, botVel.y);
   if(lastDirection2 == direction2){
      if(direction2){
        if(!flag){
          if(traj)
            delete traj;
          traj = BallInterception::getIntTraj(start, ballPos, ballVel, botVel);
          algoController = new ControllerWrapper(traj, botVel.x, botVel.y,  PREDICTION_PACKET_DELAY);
        }
        else{
          delayedVel = algoController->getDelayedVel();
          start = predictedPoseQ.front();
          
          if(traj) 
            delete traj;
          traj = BallInterception::getIntTraj(start, ballPos, ballVel, delayedVel);
          algoController = new ControllerWrapper(traj, delayedVel.x, delayedVel.y, PREDICTION_PACKET_DELAY);
        }
      }
      else {
        if(!flag){
          if(traj)
            delete traj;
          Vector2D<float> botVel2((-1)*botVel.y , (-1)*botVel.x); 
          traj = BallInterception::getIntTraj(start2, ballPos, ballVel, botVel2);
          algoController = new ControllerWrapper(traj, botVel2.x, botVel2.y, PREDICTION_PACKET_DELAY);
        }
        else{
          delayedVel = algoController->getDelayedVel();
          
          start2 = predictedPoseQ.front();
          if(traj)
            delete traj;
          traj = BallInterception::getIntTraj(start2, ballPos, ballVel, delayedVel);
          algoController = new ControllerWrapper(traj, delayedVel.x, delayedVel.y, PREDICTION_PACKET_DELAY);
        }
      }
    }
    else{
      if(direction2){
        if(!flag){
          if(traj)
            delete traj;
          traj = BallInterception::getIntTraj(start, ballPos, ballVel, botVel);
          algoController = new ControllerWrapper(traj, botVel.y, botVel.x,  PREDICTION_PACKET_DELAY);
        }
        else{
          delayedVel = algoController->getDelayedVel();
          Vector2D<float> delayedVel2((-1)*delayedVel.y , (-1)*delayedVel.x);
          Pose startT = predictedPoseQ.front();
          //startT.setTheta(normalizeAngle(start.theta() - PI));
          if(traj) 
            delete traj;
          traj = BallInterception::getIntTraj(startT, ballPos, ballVel, delayedVel2);
          algoController = new ControllerWrapper(traj, (-1)*delayedVel.y, (-1)*delayedVel.x, PREDICTION_PACKET_DELAY);
        }
      }
      else {
        if(!flag){
          if(traj)
            delete traj;
          Vector2D<float> botVel2((-1)*botVel.y , (-1)*botVel.x);   
          traj = BallInterception::getIntTraj(start2, ballPos, ballVel, botVel2);
          algoController = new ControllerWrapper(traj, (-1)*botVel.y, (-1)*botVel.x, PREDICTION_PACKET_DELAY);
        }
        else{
        //  cout << "here" << endl;
          delayedVel = algoController->getDelayedVel();
          Pose start2T = predictedPoseQ.front();
          //start2T.setTheta(normalizeAngle(start.theta() - PI));
          Vector2D<float> delayedVel2((-1)*delayedVel.y , (-1)*delayedVel.x);
          
           //delayed vel to be reversed
           //Vector2D<float> delayedVel2((-1)*delayedVel.y, (-1)*delayedVel.x);
          if(traj)
            delete traj;
          traj = BallInterception::getIntTraj(start2T, ballPos, ballVel, delayedVel2);
          algoController = new ControllerWrapper(traj, (-1)*delayedVel.y, (-1)*delayedVel.x, PREDICTION_PACKET_DELAY);
        }
      }
    }
    lastDirection2 = direction2 ; 
    _splineInterceptBallTrack(botID, start, ballPos, ballVel, botVel, final_vl, final_vr);
  }
  
  void SkillSet::splineInterceptBall(const SParam& param)
  {
    float final_vl = param.SplineInterceptBallP.vl;
    float final_vr = param.SplineInterceptBallP.vr;
    Vector2D<float> ballPos, ballVel, botVel;
    Pose start(state->homePos[botID].x, state->homePos[botID].y, state->homeAngle[botID]);
      
    //cout << counter << endl;
    ballPos.x = state->ballPos.x;
    ballPos.y = state->ballPos.y;
    if(param.SplineInterceptBallP.velGiven == 0){
      ballVel.x = state->ballVel.x;
      ballVel.y = state->ballVel.y;
    }
    else{
      ballVel.x = param.SplineInterceptBallP.ballVelX;
      ballVel.y = param.SplineInterceptBallP.ballVelY;
    }
    botVel.x = state->homeVlVr[botID].x;
    botVel.y = state->homeVlVr[botID].y;
    double dt = 10;
    if(traj){
      SplineTrajectory *st = dynamic_cast<SplineTrajectory*>(traj);
      dt = st->totalTime() - algoController->getCurrentTimeS();
    }
    if(param.SplineInterceptBallP.initTraj == 1 || dt < 0.075)
       _splineInterceptBallInitTraj(botID, start, ballPos, ballVel, botVel, final_vl, final_vr, 0);
    if (param.SplineInterceptBallP.changeSpline == 1 && interceptCounter > 60) 
        _splineInterceptBallInitTraj(botID, start, ballPos, ballVel, botVel, final_vl, final_vr, 1);
    else {
        _splineInterceptBallTrack(botID, start, ballPos, ballVel, botVel, final_vl, final_vr);
    }
  }
}