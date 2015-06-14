#include "skillSet.h"
#include "beliefState.h"
#include "logger.h"
#include "config.h"
#include "pose.h"
#include "trajectory-generators.cpp"
#include <iostream>

#define PREDICTION_PACKET_DELAY 4
static bool direction = true;
using namespace Util;

namespace Strategy
{
	
	bool SkillSet::_isFrontDirected(Pose botPos, Pose endPos) {
		int r = 10;
		double cosTheta = cos(botPos.theta());
		double sinTheta = sin(botPos.theta());
		double testx = botPos.x() + r * cosTheta;
		double testy = botPos.y() + r * sinTheta;
		double v1 = testx / tan(botPos.theta()) + testy - botPos.x() / tan(botPos.theta()) - botPos.y();
		double v2 = endPos.x() / tan(botPos.theta()) + endPos.y() - botPos.x() / tan(botPos.theta()) - botPos.y();
		return ((v1 * v2) >= 0) ? true : false;
	}
		
	void SkillSet::_splineGoToPointTrack(int botid, Pose start, Pose end, float finalvel){
		
		counter++;	
		if(!algoController){
			if(traj)
				algoController = new ControllerWrapper(traj, 0, 0, PREDICTION_PACKET_DELAY);
		}
		int vl,vr;
		if(!direction)
			start.setTheta(start.theta() - PI);
		algoController->genControls(start, end, vl, vr, finalvel);
		assert(vl <= 120 && vl >= -120);
		assert(vr <= 120 && vr >= -120);
		if (direction)
			comm->sendCommand(botid, vl, vr); //maybe add mutex
		else {
			//cout << "ulta chaloooo" << endl;
			int vl1 = (-1)*vr;
			int vr1 = (-1)*vl;
			comm->sendCommand(botid, vl1, vr1);
		}
	}

	void SkillSet::_splineGoToPointInitTraj(int botid, Pose start, Pose end, float finalvel){
		counter = 0;
		if(traj)delete traj;
		direction = _isFrontDirected(start, end);
		if (!direction) 
			start.setTheta(start.theta() - PI);
		traj = TrajectoryGenerators::cubic(start, end ,0,0,0,0); //may need to modify vle,vls,vre,vrs
		
		if(algoController)delete algoController;
		algoController = new ControllerWrapper(traj, 0, 0, PREDICTION_PACKET_DELAY);
		
		/*while(!predictedPoseQ.empty())predictedPoseQ.pop_front();
		for (int i = 0; i < PREDICTION_PACKET_DELAY; i++) {
			predictedPoseQ.push_back(Pose(bs.homeX[BOT_ID_TESTING], bs.homeY[BOT_ID_TESTING], bs.homeTheta[BOT_ID_TESTING]));
		}*/
		_splineGoToPointTrack(botid,start,end,finalvel);
	}
	
	void SkillSet::_splineGoToPointCheckTraj(int botid, Pose start, Pose end, float finalvel, int initTraj){

		if(initTraj==1)
			_splineGoToPointInitTraj(botID, start, end, finalvel);
			
		if(counter < 100)
			_splineGoToPointTrack(botid,start,end,finalvel);
		
		
		if(counter > 100){
			delete traj;
			counter = 0;
		}
		int vl,vr;
		algoController->genControls(start, end, vl, vr, finalvel);		
		direction = _isFrontDirected(start, end);
		if (!direction) 
			start.setTheta(start.theta() - PI);
		traj = TrajectoryGenerators::cubic(start, end ,vl,vr,0,0); //may need to modify vle,vls,vre,vrs
		
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
	Pose end(param.SplineGoToPointP.x, param.SplineGoToPointP.y, param.SplineGoToPointP.finalSlope);
	_splineGoToPointCheckTraj(botID, start, end, finalvel, param.SplineGoToPointP.initTraj);
	
//	if(param.SplineGoToPointP.initTraj)_splineGoToPointInitTraj(botID, start, end, finalvel);
//	else _splineGoToPointTrack(botID, start, end, finalvel);
	
   }
}