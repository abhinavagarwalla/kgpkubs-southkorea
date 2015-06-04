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
		
	void SkillSet::_splineGoToPointTrack(int botid, Pose start, Pose end, float finalvel){

		if(!algoController){
			if(traj)
				algoController = new ControllerWrapper(traj, 0, 0, PREDICTION_PACKET_DELAY);
		}
		int vl,vr;
		algoController->genControls(start, end, vl, vr, finalvel);
		assert(vl <= 120 && vl >= -120);
		assert(vr <= 120 && vr >= -120);
		comm->sendCommand(botid, vl, vr); //maybe add mutex
	}

	void SkillSet::_splineGoToPointInitTraj(int botid, Pose start, Pose end, float finalvel){

		if(traj)delete traj;
		traj = TrajectoryGenerators::cubic(start, end ,0,0,0,0); //may need to modify vle,vls,vre,vrs
		
		if(algoController)delete algoController;
		algoController = new ControllerWrapper(traj, 0, 0, PREDICTION_PACKET_DELAY);
		
		/*while(!predictedPoseQ.empty())predictedPoseQ.pop_front();
		for (int i = 0; i < PREDICTION_PACKET_DELAY; i++) {
			predictedPoseQ.push_back(Pose(bs.homeX[BOT_ID_TESTING], bs.homeY[BOT_ID_TESTING], bs.homeTheta[BOT_ID_TESTING]));
		}*/
		_splineGoToPointTrack(botid,start,end,finalvel);
	}
	
  void SkillSet::splineGoToPoint(const SParam& param)
  {
	Vector2D<int> dpoint;
    float finalvel;
    finalvel  = param.GoToPointP.finalVelocity;
	Pose start(state->homePos[botID].x, state->homePos[botID].y, state->homeAngle[botID]);
	Pose end(param.SplineGoToPointP.x, param.SplineGoToPointP.y, param.SplineGoToPointP.finalslope);
	
	if(param.SplineGoToPointP.initTraj)_splineGoToPointInitTraj(botID, start, end, finalvel);
	else _splineGoToPointTrack(botID, start, end, finalvel);
	
   }
}