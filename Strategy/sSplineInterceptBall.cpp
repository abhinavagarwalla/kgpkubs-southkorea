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
		
	//	cout << "in this" << endl;
		interceptCounter++;
		if(!algoController){
			if(traj)
				algoController = new ControllerWrapper(traj, state->homeVlVr[botID].x, state->homeVlVr[botID].y, PREDICTION_PACKET_DELAY);
		}
		int vl,vr;
		Pose dummy(0,0,0);
		algoController->genControls(start, dummy, vl, vr, 0);
		assert(vl <= 150 && vl >= -150);
		assert(vr <= 150 && vr >= -150);
		if (direction)
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
		
		if(dist < 50){
			comm->sendCommand(botID, 0, 0);
			return;
		}
		cout << ballVel.x << " " << ballVel.y << endl;
		//getchar();
		Pose start2(state->homePos[botID].x, state->homePos[botID].y, normalizeAngle(state->homeAngle[botID] - PI));
		//direction = _isFrontDirected(start, Pose(state->ballPos.x, state->ballPos.y , Vector2D<int>::angle(state->homePos[botID], state->ballPos)), state->homeVlVr[botID].x, state->homeVlVr[botID].y);
		interceptCounter = 0;	
		Vector2D<float> delayedVel(botVel.x, botVel.y);
		if(direction){
			if(!flag){
				if(traj)
					delete traj;
				traj = BallInterception::getIntTraj(start, ballPos, ballVel, botVel);
			}
			else{
				delayedVel = algoController->getDelayedVel();
				Pose start = algoController->getNewStartPose();
				
				if(traj) 
					delete traj;
				traj = BallInterception::getIntTraj(start, ballPos, ballVel, delayedVel);
			}
		}
		else {
			Vector2D<float> botVel2((-1)*botVel.y , (-1)*botVel.x);
			if(!flag){
				if(traj)
					delete traj;
				traj = BallInterception::getIntTraj(start2, ballPos, ballVel, botVel2);
			}
			else{
			//	cout << "here" << endl;
				delayedVel = algoController->getDelayedVel();
				Pose start2 = algoController->getNewStartPose();

				if(traj)
					delete traj;
				traj = BallInterception::getIntTraj(start, ballPos, ballVel, delayedVel);
			}
		}
		
		if(algoController)
			delete algoController;
		
		cout << state->homeVlVr[botID].x << " " << state->homeVlVr[botID].y << endl;
		if(direction)
			algoController = new ControllerWrapper(traj, state->homeVlVr[botID].x, state->homeVlVr[botID].y, PREDICTION_PACKET_DELAY);
		else
			algoController = new ControllerWrapper(traj, (-1)*state->homeVlVr[botID].y, (-1)*state->homeVlVr[botID].x, PREDICTION_PACKET_DELAY);
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
			
		cout << counter << endl;
		ballPos.x = state->ballPos.x;
		ballPos.y = state->ballPos.y;
		ballVel.x = state->ballVel.x;
		ballVel.y = state->ballVel.y;
		botVel.x = state->homeVlVr[botID].x;
		botVel.y = state->homeVlVr[botID].y;
		
		if(param.SplineInterceptBallP.initTraj == 1)
			_splineInterceptBallInitTraj(botID, start, ballPos, ballVel, botVel, final_vl, final_vr, 0);
		if(interceptCounter > 30000)
			_splineInterceptBallInitTraj(botID, start, ballPos, ballVel, botVel, final_vl, final_vr, 1);
		else 
			_splineInterceptBallTrack(botID, start, ballPos, ballVel, botVel, final_vl, final_vr);	
	}
}