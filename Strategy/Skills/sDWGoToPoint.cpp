#include "skillSet.h"
#include "beliefState.h"
#include "logger.h"
#include "config.h"
#include "pose.h"
#include <iostream>

#define PREDICTION_PACKET_DELAY 4
using namespace Util;

namespace Strategy
{
    
  void SkillSet::_dwGoToPointTrack(int botid, Pose start, Pose end, float finalvel){

    if(!algoController){
      algoController = new ControllerWrapper(&Controllers::DynamicWindow, 0, 0, PREDICTION_PACKET_DELAY);
      
    }
    int vl,vr;
    algoController->genControls(start, end, vl, vr, finalvel);
    assert(vl <= 120 && vl >= -120);
    assert(vr <= 120 && vr >= -120);
    comm->sendCommand(botid, vl, vr); //maybe add mutex
  }

  void SkillSet::_dwGoToPointInitTraj(int botid, Pose start, Pose end, float finalvel){
    
    if(algoController)delete algoController;
    algoController = new ControllerWrapper(&Controllers::DynamicWindow, 0, 0, PREDICTION_PACKET_DELAY);
  
    _dwGoToPointTrack(botid,start,end,finalvel);
  }
  
  void SkillSet::dwGoToPoint(const SParam& param)
  {
  Vector2D<int> dpoint;
    float finalvel;
    finalvel  = param.GoToPointP.finalVelocity;
  Pose start(state->homePos[botID].x, state->homePos[botID].y, state->homeAngle[botID]);
  Pose end(param.DWGoToPointP.x, param.DWGoToPointP.y, param.DWGoToPointP.finalslope);
  
  if(param.DWGoToPointP.initTraj)_dwGoToPointInitTraj(botID, start, end, finalvel);
  else _dwGoToPointTrack(botID, start, end, finalvel);
  
   }
}