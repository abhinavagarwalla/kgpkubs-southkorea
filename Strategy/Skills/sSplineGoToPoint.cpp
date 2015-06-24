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
		//std::cout << "here v cnds" << std::endl;
		if(!algoController){
			if(traj)
				algoController = new ControllerWrapper(traj, vls, vrs, PREDICTION_PACKET_DELAY);
		}
		
	//	cout << "flag is " << start.x() << " " << start.y() << endl ;
		int vl,vr;
		if(!direction)
			start.setTheta(normalizeAngle(start.theta() - PI));
			
		algoController->genControls(start, end, vl, vr, finalvel);
		assert(vl <= 120 && vl >= -120);
		assert(vr <= 120 && vr >= -120);
		cout << "cfsw" << counter << " " << vr << " " << vl << endl;
		if (direction)
			comm->sendCommand(botid, vl/2, vr/2); //maybe add mutex
		else {
			//cout << "ulta chaloooo" << endl;
			int vl1 = (-1)*vr;
			int vr1 = (-1)*vl;
			comm->sendCommand(botid, vl1/2, vr1/2);
		}
	}

	void SkillSet::_splineGoToPointInitTraj(int botid, Pose start, Pose end, float finalvel, float vls, float vrs, int flag){
	
		counter = 0;
		direction = _isFrontDirected(start, end, vls, vrs);
		if (!direction) 
			start.setTheta(start.theta() - PI);
		cout << "flag is " << start.x() << " " << start.y() << endl ;
	//	getchar();
/*		
		if(flag == 1){
			 pair<int,int> delayedVel = algoController->getDelayedVel();
		     start = algoController->getNewStartPose();
			 if(traj)
				delete traj;
			 traj = TrajectoryGenerators::cubic(start, end ,delayedVel.first, delayedVel.second,0,0);
		}
		else{*/
			if(traj)
				delete traj;
			traj = TrajectoryGenerators::cubic(start, end ,  0, 0 ,0,0); //may need to modify vle,vls,vre,vrs
		//}
		
		if(algoController)
			delete algoController;
		
		algoController = new ControllerWrapper(traj, vls, vrs, PREDICTION_PACKET_DELAY);

		_splineGoToPointTrack(botid,start,end,finalvel, vls, vrs);
	}
  void SkillSet::splineGoToPoint(const SParam& param)
  {
	Vector2D<int> dpoint;
    float finalvel;
	// std::cout <<param.SplineGoToPointP.initTraj << "inittraj"  << std::endl;
    finalvel  = param.GoToPointP.finalVelocity;
	Pose start(state->homePos[botID].x, state->homePos[botID].y, state->homeAngle[botID]);
	Pose end(param.SplineGoToPointP.x, param.SplineGoToPointP.y, param.SplineGoToPointP.finalSlope);
	if(param.SplineGoToPointP.initTraj == 1){
		_splineGoToPointInitTraj(botID, start, end, finalvel, state->homeVlVr[botID].x, state->homeVlVr[botID].y, 0);
	}
	else if(counter > 20000){
		_splineGoToPointInitTraj(botID, start, end, finalvel, state->homeVlVr[botID].x, state->homeVlVr[botID].y, 1);
	}
	else 
		_splineGoToPointTrack(botID, start, end, finalvel, state->homeVlVr[botID].x, state->homeVlVr[botID].y);
	//cout << "lksdhbvkiedvi;oe;lqwibdkjs bvak.jb jcl; bvkjola;b fd;";
   }
}