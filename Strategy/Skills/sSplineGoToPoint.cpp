#include "skillSet.h"
#include "beliefState.h"
#include "logger.h"
#include "config.h"
#include "pose.h"
#include "trajectory-generators.cpp"
#include <iostream>
#include <fstream>

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
		assert(vl <= 150 && vl >= -150);
		assert(vr <= 150 && vr >= -150);
	//	cout << "sending packet" << counter << " " << vr << " " << vl << endl;
	//	std::ofstream outfile;
	//	outfile.open("test.txt", std::ios_base::app);
		//outfile << "sending packet" << counter << " " << vr << " " << vl << endl;
		if (direction){
			comm->sendCommand(botid, vl, vr); //maybe add mutex
		}else {
			//cout << "ulta chaloooo" << endl;
			int vl1 = (-1)*vr;
			int vr1 = (-1)*vl;
			comm->sendCommand(botid, vl1, vr1);
		}
	}

	void SkillSet::_splineGoToPointInitTraj(int botid, Pose start, Pose end, float finalvel, float vls, float vrs, int flag){
	
		counter = 0;
		Pose start2(state->homePos[botID].x, state->homePos[botID].y, normalizeAngle(state->homeAngle[botID] - PI));
		pair<int,int> delayedVel = make_pair(vls, vrs);
		direction = _isFrontDirected(start, end, vls, vrs);
		if(direction){
			if(!flag){
				if(traj)
					delete traj;
				traj = TrajectoryGenerators::cubic(start, end, vls, vrs, 0, 0);
			}
			else{
			//	cout << "here" << endl;
				delayedVel = algoController->getDelayedVel();
				Pose start = algoController->getNewStartPose();
				if(traj) 
					delete traj;
				traj = TrajectoryGenerators::cubic(start, end, delayedVel.first , delayedVel.second , 0, 0);
			}
		}
		else {
			if(!flag){
				if(traj)
					delete traj;
				traj = TrajectoryGenerators::cubic(start2, end, vls, vrs, 0, 0);
			}
			else{
			//	cout << "here" << endl;
				delayedVel = algoController->getDelayedVel();
				Pose start2 = algoController->getNewStartPose();
				if(traj)
					delete traj;
				traj = TrajectoryGenerators::cubic(start2, end, delayedVel.first, delayedVel.second, 0, 0);
			}
		}
		
		if(algoController)
			delete algoController;
		
		
		algoController = new ControllerWrapper(traj, delayedVel.first , delayedVel.second , PREDICTION_PACKET_DELAY);

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
	else if(counter > 20){
		_splineGoToPointInitTraj(botID, start, end, finalvel, state->homeVlVr[botID].x, state->homeVlVr[botID].y, 1);
	}
	else 
		_splineGoToPointTrack(botID, start, end, finalvel, state->homeVlVr[botID].x, state->homeVlVr[botID].y);
   }
}

