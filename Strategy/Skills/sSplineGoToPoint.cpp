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
	bool SkillSet::_isFrontDirected(Pose botPos, Pose endPos, float vle, float vre) {
		int r = 10;
		double cosTheta = cos(botPos.theta());
		double sinTheta = sin(botPos.theta());
		double testx = botPos.x() + r * cosTheta;
		double testy = botPos.y() + r * sinTheta;
/*		if(((vle + vre)/2 ) > 30)
			return true;
		if((( vle + vre)/2)< -30)
			return false;*/
		double v1 = testx / tan(botPos.theta()) + testy - botPos.x() / tan(botPos.theta()) - botPos.y();
		double v2 = endPos.x() / tan(botPos.theta()) + endPos.y() - botPos.x() / tan(botPos.theta()) - botPos.y();
		return ((v1 * v2) >= 0) ? true : false;
	}
		
	void SkillSet::_splineGoToPointTrack(int botid, Pose start, Pose end, float finalvel, float vls, float vrs){
		
		counter++;	
	//	std::cout << "here v cnds" << std::endl;
		if(!algoController){
			if(traj)
				algoController = new ControllerWrapper(traj, vls, vrs, PREDICTION_PACKET_DELAY);
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

	void SkillSet::_splineGoToPointInitTraj(int botid, Pose start, Pose end, float finalvel, float vls, float vrs){
	
		counter = 0;
		if(traj)delete traj;
		direction = _isFrontDirected(start, end, vls, vrs);
		if (!direction) 
			start.setTheta(start.theta() - PI);
		traj = TrajectoryGenerators::cubic(start, end ,vls,vrs,0,0); //may need to modify vle,vls,vre,vrs
		
		if(algoController)delete algoController;
		algoController = new ControllerWrapper(traj, vls, vrs, PREDICTION_PACKET_DELAY);
		
		/*while(!predictedPoseQ.empty())predictedPoseQ.pop_front();
		for (int i = 0; i < PREDICTION_PACKET_DELAY; i++) {
			predictedPoseQ.push_back(Pose(bs.homeX[BOT_ID_TESTING], bs.homeY[BOT_ID_TESTING], bs.homeTheta[BOT_ID_TESTING]));
		}*/
		_splineGoToPointTrack(botid,start,end,finalvel, vls, vrs);
	}
  void SkillSet::splineGoToPoint(const SParam& param)
  {
	Vector2D<int> dpoint;
    float finalvel;
	 std::cout << "counter " << state->homeVlVr[botID].x << " " << state->homeVlVr[botID].y  << std::endl;
    finalvel  = param.GoToPointP.finalVelocity;
	Pose start(state->homePos[botID].x, state->homePos[botID].y, state->homeAngle[botID]);
	Pose end(param.SplineGoToPointP.x, param.SplineGoToPointP.y, param.SplineGoToPointP.finalSlope);
	if(param.SplineGoToPointP.initTraj == 1 || counter > 50){
		_splineGoToPointInitTraj(botID, start, end, finalvel, state->homeVlVr[botID].x, state->homeVlVr[botID].y);
	}
	else 
		_splineGoToPointTrack(botID, start, end, finalvel, state->homeVlVr[botID].x, state->homeVlVr[botID].y);
	
   }
}