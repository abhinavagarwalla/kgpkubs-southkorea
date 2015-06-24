#include "skillSet.h"
#include "beliefState.h"
#include "logger.h"
#include "config.h"
#include "pose.h"
#include <iostream>
#include "ballinterception.cpp"

#define PREDICTION_PACKET_DELAY 4
using namespace Util;
static bool direction = true;

namespace Strategy
{
		
	void SkillSet::_splineInterceptBallTrack(int botid, Pose start, Vector2D<float> ballPos, Vector2D<float> ballVel, Vector2D<float> botVel, float final_vl, float final_vr){
		
		interceptCounter++;
		if(!algoController){
			if(traj)
				algoController = new ControllerWrapper(traj, state->homeVlVr[botID].x, state->homeVlVr[botID].y, PREDICTION_PACKET_DELAY);
		}
		int vl,vr;
		Pose dummy(0,0,0);
		algoController->genControls(start, dummy, vl, vr, 60);
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

	void SkillSet::_splineInterceptBallInitTraj(int botid, Pose start, Vector2D<float> ballPos, Vector2D<float> ballVel, Vector2D<float> botVel, float final_vl, float final_vr){

		direction = _isFrontDirected(start, Pose(state->ballPos.x, state->ballPos.y , Vector2D<int>::angle(state->homePos[botID], state->ballPos)), state->homeVlVr[botID].x, state->homeVlVr[botID].y);
		
		interceptCounter = 0;	
		if(traj)delete traj;
		
		if (!direction) 
			start.setTheta(start.theta() - PI);
		traj = BallInterception::getIntTraj(start, ballPos, ballVel, botVel); //may need to modify vle,vls,vre,vrs
		
		if(algoController)delete algoController;
		//algoController = new ControllerWrapper(traj, 0, 0, PREDICTION_PACKET_DELAY);
		algoController = new ControllerWrapper(traj, state->homeVlVr[botID].x, state->homeVlVr[botID].y, PREDICTION_PACKET_DELAY);
		/*while(!predictedPoseQ.empty())predictedPoseQ.pop_front();
		for (int i = 0; i < PREDICTION_PACKET_DELAY; i++) {
			predictedPoseQ.push_back(Pose(bs.homeX[BOT_ID_TESTING], bs.homeY[BOT_ID_TESTING], bs.homeTheta[BOT_ID_TESTING]));
		}*/
		_splineInterceptBallTrack(botID, start, ballPos, ballVel, botVel, final_vl, final_vr);
	}
	
	void SkillSet::splineInterceptBall(const SParam& param)
	{
		float final_vl = param.SplineInterceptBallP.vl;
		float final_vr = param.SplineInterceptBallP.vr;
		Vector2D<float> ballPos, ballVel, botVel;
		Pose start(state->homePos[botID].x, state->homePos[botID].y, state->homeAngle[botID]);
		
		if (!direction) 
			start.setTheta(start.theta() - PI);
			
		cout << counter << endl;
		ballPos.x = state->ballPos.x;
		ballPos.y = state->ballPos.y;
		ballVel.x = state->ballVel.x;
		ballVel.y = state->ballVel.y;
		botVel.x = state->homeVlVr[botID].x;
		botVel.y = state->homeVlVr[botID].y;
		if(param.SplineInterceptBallP.initTraj || interceptCounter > 10000)_splineInterceptBallInitTraj(botID, start, ballPos, ballVel, botVel, final_vl, final_vr);
		else _splineInterceptBallTrack(botID, start, ballPos, ballVel, botVel, final_vl, final_vr);	
	}
}