#include <sys/time.h>
#include "comdef.h"
#include "kalman.h"
#include "cs.hpp"
#include "beliefState.h"
#include "messages_robocup_ssl_detection.pb.h"
#include "rbCommDef.h"
#include "config.h"
#include "logger.h"
#include <limits>
#include <set>
#include <iostream>
#include <fstream>

#define DEBUG_KALMAN 1
const int botcenterbluecenterdist =75; //Ankit: for botCentreTransform
bool isIndeterminate(const float pV)
{
    volatile float d = pV;
    return (d != d);
} 
inline void linearTransform(int &x, int &y, float &newangle)
{
  int tempx, tempy;
  tempx = (y*(-HALF_FIELD_MAXX))/(2050.0);
  tempy = ((x + 3025.0/2)*(HALF_FIELD_MAXY))/(3025.0/2.0);
  x = tempx;
  y = tempy;
  newangle = normalizeAngle(newangle+PI/2);
  //y=tempy-800;
}
inline void botcenterTransform(int &x, int &y, float &newangle)//ssl vision detects blue center as bot center
{
  /// Commented. Not needed currently.
  // x=x-botcenterbluecenterdist*cos(newangle+3*PI/4);
  //y=y-botcenterbluecenterdist*sin(newangle+3*PI/4);
}
bool isInfinite(const float pV)
{
    return (fabs(pV) == std::numeric_limits<float>::infinity());
}
void checkValid(float &x, float resetVal) {
  if(isIndeterminate(x) || isInfinite(x)) {
    assert(0);
    x =resetVal;
  }
}
void checkValidX(float &x, float &slope, float resetVal) {
  if(x>=-2*HALF_FIELD_MAXX && x<=2*HALF_FIELD_MAXX)return;
  x = resetVal;
  slope = 0;
  checkValid(x, resetVal);
}
void checkValidY(float &y, float &slope, float resetVal) {
  if(y>=-2*HALF_FIELD_MAXY && y<=2*HALF_FIELD_MAXY)return;
  y = resetVal;
  slope = 0;
  checkValid(y, resetVal);
}
void checkValidA(float &a, float &slope, float resetVal) {
  if(a>=-2*PI && a<=2*PI)return;
  a = resetVal;
  slope = 0;
  checkValid(a, resetVal);
}
namespace Strategy
{
  Kalman::Kalman()
  {
    kalmanlog = fopen("kalman.log", "w");
    t.start();
    mutex = new Util::CS();
    for(int id = 0; id < HomeTeam::SIZE; ++id)
    {
      homePose[id].x     = 2*HALF_FIELD_MAXX + BOT_RADIUS*4;
      homePose[id].y     = 2*HALF_FIELD_MAXY + BOT_RADIUS*4;
      homeVelocity[id].x = 0;
      homeVelocity[id].y = 0;
	  homeVlVr[id].x = 0;
      homeVlVr[id].y = 0;
      homePosK[id].x     = 1.0;
      homePosK[id].y     = 1.0;
      homeAngle[id]      = 0;
      homeOmega[id]      = 0;
      homeAngleK[id]     = 1;
    }
    for(int id = 0; id < AwayTeam::SIZE; id++)
    {
      awayPose[id].x     = 2*HALF_FIELD_MAXX + BOT_RADIUS*4;
      awayPose[id].y     = 2*HALF_FIELD_MAXY + BOT_RADIUS*4;
      awayVelocity[id].x = 0;
      awayVelocity[id].y = 0;
      awayPosK[id].x     = 1;
      awayPosK[id].y     = 1;
      awayAngle[id]      = 0;
      awayOmega[id]      = 0;
      awayAngleK[id]     = 1;
    }
    ballPose.x     = 0;
    ballPose.y     = 0;
    ballVelocity.x = 0;
    ballVelocity.y = 0;
    ballPosK.x     = 1;
    ballPosK.y     = 1;
	
	 for(int i = 0 ; i<=5;i++){
		 QueueVel.push_back(Vector2D<float>(0,0));
		 QueuePos.push_back(Vector2D<float>(0,0));
		}
	 
	  myfile.open ("exmple.txt");
	  
	for (int i = 0; i < MAX_BS_Q; i++) {
        bsQ.push(std::make_pair(BeliefState(), 0));
    }
	for(int i = 1 ; i<5;i++)
		ballPosQueue.push_back(Vector2D<float>(0,0));
  }

  Kalman::~Kalman()
  {
	myfile.close();
    fclose(kalmanlog);
  }
	int Kalman::getClosestBotID(int x, int y, float angle, std::set<int> &uniqueBotIDs)
	{		
		int closest = -1;
		float closestCost = 9999999999.0;
		for (int botID = 0; botID < AwayTeam::SIZE; ++botID)
    {
			float cost = (x - awayPose[botID].x)*(x - awayPose[botID].x);
			cost += (y - awayPose[botID].y)*(y - awayPose[botID].y);
			cost += (angle - awayAngle[botID])*(angle - awayAngle[botID]);
			if(cost < closestCost && uniqueBotIDs.find(botID) == uniqueBotIDs.end()) {
				closest = botID;
				closestCost = cost;
			}				
    }
		return closest; //returns -1 if no suitable bot present.
	}
  static float nearestAngle(float angle, float reference)
  {
    float del = fabs(angle-reference);
    float res = angle;
    if(del > fabs(angle-reference+2*PI))
    {
      del = fabs(angle-reference+2*PI);
      res = angle+2*PI;
    }
    if(del > fabs(angle-reference-2*PI))
    {
      res = angle-2*PI;
    }
    return res;
  }

  void Kalman::strategyToRealConversion(BotPose &p) {
    // converts from strategy to actual coordinates (cm)
    p.x /= fieldXConvert;
    p.y /= fieldYConvert;
  }
  
  Vector2D<float> Kalman::calcBotVelocity(BotPose p1, BotPose p2, float timeMs) {
    strategyToRealConversion(p1);
    strategyToRealConversion(p2);

	float vl,vr;
    double delX = p2.x - p1.x;
    double delY = p2.y - p1.y;
    double delTheta = normalizeAngle(p2.theta - p1.theta); // assuming |delTheta| < PI, which is safe to assume
                                                           // for ~ 16 ms of rotation at any speed (even 120,-120?).
    assert(timeMs > 0);
    // w * timeMs = delTheta
    double w = delTheta / (timeMs * 0.001);
    if (delTheta < 1e-2 && delTheta > -1e-2) {  // be realistic
        // bot should be headed straight, but again confusion
        // so taking projection of (delX, delY) along (cos(theta), sin(theta)) as displacement.
        double dispLength = delX*cos(p1.theta) + delY*sin(p1.theta);
        vl = dispLength / (timeMs * 0.001);
        vl = vl / ticksToCmS;
        vr = vl;
		Vector2D<float> v(vl,vr);
        return v;
    }
    // we calculate 2 rho's, based on delX and delY, and take average
    double rho1 = delX / (sin(p2.theta) - sin(p1.theta));
    double rho2 = -delY / (cos(p2.theta) - cos(p1.theta));
    // try harmonic mean?
//    double rho = (rho1 + rho2) / 2;
    double rho = 2*rho1*rho2 / (rho1 + rho2);
    vl = w * (rho - d/2.0) / ticksToCmS;
    vr = w * (rho + d/2.0) / ticksToCmS;

	Vector2D<float> v(vl,vr);
    return v;
  }

	
  void Kalman::addInfo(SSL_DetectionFrame &detection)
  {
    static double minDelTime = 999999;
    static double lastTime = 0;
    mutex->enter();
    //double timeCapture           = detection.t_capture();
    int ballsNum                 = detection.balls_size();
    blueNum                      = MIN(detection.robots_blue_size(), Simulator::BlueTeam::SIZE);
    yellowNum                    = MIN(detection.robots_yellow_size(), Simulator::YellowTeam::SIZE);

	//fprintf(kalmanlog, ">>>>>>>>>>>>>>>>Time: %f\n", timeCapture - lastTime);
    //double timeCapture      = detection.t_capture()/1000.0; //This is only valid if both the other side also uses the same reference time
    // neglecting the time for network delay. and wait time in race conditions 
    struct timeval tv;
    assert(gettimeofday(&tv, NULL) == 0);
    double timeCapture      = tv.tv_sec+tv.tv_usec/1000000.0;
    
    lastTime = timeCapture;
    std::set<int> uniqueBotIDs;
	
	double nowTime = detection.t_capture();
	double timeMs = (nowTime - bsQ.front().second)*1000.0;
	if(timeMs <= 0){
		timeMs = 0.001;
	}
	//Adding Ball Info
    SSL_DetectionBall ball;
    if (ballsNum > 0)
    {
      ball = detection.balls(0);
	  
      int newx = ball.x() - CENTER_X;
      int newy = ball.y() - CENTER_Y;
      double delTime = timeCapture - ballLastUpdateTime;
      float garbage;
      #if GR_SIM_COMM || FIRASSL_COMM
      linearTransform(newx, newy, garbage);
	 // std::cout << "\n\n\nhere linear\n\n\n" << std::endl;
      #endif
	  
      ballPosSigmaSqK.x          = ballPosSigmaSqK.x * ( 1 - ballPosK.x) + SIGMA_SQ_NOISE_POS * delTime;
      ballPosK.x                 = ballPosSigmaSqK.x / (ballPosSigmaSqK.x + SIGMA_SQ_OBSVN_POS);
	 
      float  predictedPoseX      = ballPose.x + ballVelocity.x * (delTime);
      float  lastPoseX           = ballPose.x;
      ballPose.x                 = predictedPoseX + ballPosK.x * (newx - predictedPoseX);
      
      ballPosSigmaSqK.y          = ballPosSigmaSqK.y * ( 1 - ballPosK.y) + SIGMA_SQ_NOISE_POS * delTime;
      ballPosK.y                 = ballPosSigmaSqK.y / (ballPosSigmaSqK.y + SIGMA_SQ_OBSVN_POS);
      float  predictedPoseY      = ballPose.y + ballVelocity.y * (delTime);
      float  lastPoseY           = ballPose.y;
      ballPose.y                 = predictedPoseY + ballPosK.y * (newy - predictedPoseY);
      
	  float lastVelocityx        = ballVelocity.x;
	  float lastVelocityy        = ballVelocity.y;
		ballPosQueue.pop_front();
	  ballPosQueue.push_back(ballPose);
	  ballVelocity.x             = (ballPose.x - lastPoseX) / delTime;
	  ballVelocity.y             = (ballPose.y - lastPoseY) / delTime;
	  //New code for ball Velocity
/*	  float sumX =0, sumY =0;
	  float prevPosX = ballPosQueue[0].x;
	  float prevPosY = ballPosQueue[0].y;
	  for(int i = 1; i <10 ;i+=2){
			sumX += (ballPosQueue[i].x - ballPosQueue[i-1].x)/delTime;
			sumY += (ballPosQueue[i].y - ballPosQueue[i-1].y)/delTime;
			prevPosX = ballPosQueue[i].x;
			prevPosY = ballPosQueue[i].y;
	  }
	  ballVelocity.x = sumX/5;
	  ballVelocity.y = sumY/5;
	  */
	  //ballVelocity.x = (newx - bsQ.front().first.ballPos.x)/(timeMs * 0.001);
	  //ballVelocity.y = (newy - bsQ.front().first.ballPos.y)/(timeMs * 0.001);
	  ballAcceleration.x         = (ballVelocity.x - lastVelocityx) / delTime;
      ballAcceleration.y         = (ballVelocity.y - lastVelocityy) / delTime;
      //printf("Ball: %f %f %f %f %lf\n", ballPose.x, ballPose.y, ballVelocity.x, ballVelocity.y, delTime);
// //       QueueVel.pop_front();
////	//	QueueVel.push_back(ballVelocity);
//		QueuePos.push_back(ballPose);
//		QueuePos.pop_front();
				
		float Xsum2=0, Xsum1=0, TimeDiffSum = 10*delTime, TimeDiffSqSum = 30*delTime*delTime, n=4, lambda = 0.01, Ysum2=0,Ysum1=0;

		for(int i=3;i>=0;i--)
		{
			Xsum1+=(delTime*(4-i)*ballPosQueue[i].x);
			Xsum2+=(ballPosQueue[i].x);
			Ysum1+=(delTime*(4-i)*ballPosQueue[i].y);
			Ysum2+=(ballPosQueue[i].y);
		}

		//std::cout << n*Xsum1 << " dbsakejb" << TimeDiffSum*Xsum2 << std::endl;
		int Xv_lambda = ((n*Xsum1 -TimeDiffSum*Xsum2)/(n*(lambda + TimeDiffSqSum) - TimeDiffSum*TimeDiffSum));
		int Yv_lambda = ((n*Ysum1 -TimeDiffSum*Ysum2)/(n*(lambda + TimeDiffSqSum) - TimeDiffSum*TimeDiffSum));
	
		myfile  << Xv_lambda << "\t" << Yv_lambda << "\t" << ballVelocity.x << "\t" << ballVelocity.y << std::endl;
     	ballVelocity.x = -Xv_lambda;
		ballVelocity.y = -Yv_lambda;
		checkValidX(ballPose.x, ballVelocity.x, newx);
      checkValidY(ballPose.y, ballVelocity.y, newy);
      ballLastUpdateTime         = timeCapture;
    }

    if (HomeTeam::COLOR == Simulator::BLUE_TEAM)
    {
      // Blue robot info
      
      for (int i = 0; i < blueNum; ++i)
      {
        SSL_DetectionRobot robot = detection.robots_blue(i);
		
		//Home Blue Bot Info
        int                id    = HAL::BlueMarkerMap[robot.robot_id()];
        
        int newx = robot.x() - CENTER_X;
        int newy = robot.y() - CENTER_Y;
        float newangle = robot.orientation();
//        printf("robot %d: %d %d %f\n", id, newx, newy, newangle);
        if(uniqueBotIDs.find(id) != uniqueBotIDs.end())
          continue;
        uniqueBotIDs.insert(id);
       // printf("newxy : %d %d\n", newx, newy);
        
        #if GR_SIM_COMM || FIRASSL_COMM
        linearTransform(newx, newy, newangle);
        #endif
        #if FIRASSL_COMM
        botcenterTransform(newx, newy, newangle);
        #endif
        double           delTime = timeCapture - homeLastUpdateTime[id];
        static float lastnx = 0;
        homePosSigmaSqK[id].x    = homePosSigmaSqK[id].x * ( 1 - homePosK[id].x) + SIGMA_SQ_NOISE_POS * delTime;
        assert(homePosSigmaSqK[id].x >= 0);
        homePosK[id].x           = homePosSigmaSqK[id].x / (homePosSigmaSqK[id].x + SIGMA_SQ_OBSVN_POS);
        float  predictedPoseX    = homePose[id].x + homeVelocity[id].x * (delTime);
        float  lastPoseX         = homePose[id].x;
        homePose[id].x            = predictedPoseX + homePosK[id].x * (newx - predictedPoseX);
        float lastVelocityx      = homeVelocity[id].x;
        homeVelocity[id].x       = (homePose[id].x - lastPoseX) / delTime;
        homeAcc[id].x            = (homeVelocity[id].x - lastVelocityx) / delTime;
        if(delTime < minDelTime)
          minDelTime = delTime;
//        assert(delTime > 0.00000001);
 //       printf("minDelTime = %lf\n", minDelTime);
        homePosSigmaSqK[id].y    = homePosSigmaSqK[id].y * ( 1 - homePosK[id].y) + SIGMA_SQ_NOISE_POS * delTime;
        assert(homePosSigmaSqK[id].y >= 0);
        homePosK[id].y           = homePosSigmaSqK[id].y / (homePosSigmaSqK[id].y + SIGMA_SQ_OBSVN_POS);
        float  predictedPoseY    = homePose[id].y + homeVelocity[id].y * (delTime);
        float  lastPoseY         = homePose[id].y;
        homePose[id].y           = predictedPoseY + homePosK[id].y * (newy - predictedPoseY);
        float lastVelocityy      = homeVelocity[id].y;
        homeVelocity[id].y       = (homePose[id].y - lastPoseY) / delTime;
        homeAcc[id].y            = (homeVelocity[id].y - lastVelocityy) / delTime;
                                
        homeAngleSigmaSqK[id]    = homeAngleSigmaSqK[id] * ( 1 - homeAngleK[id]) + SIGMA_SQ_NOISE_ANGLE * delTime;
        homeAngleK[id]           = homeAngleSigmaSqK[id] / (homeAngleSigmaSqK[id] + SIGMA_SQ_OBSVN_ANGLE);
        float  predictedAngle    = homeAngle[id] + homeOmega[id] * (delTime);
        float  lastAngle         = homeAngle[id];
        homeAngle[id]            = predictedAngle + homeAngleK[id] * (nearestAngle(newangle, homeAngle[id]) - predictedAngle);
        float lastAngularV       = homeOmega[id];
        homeOmega[id]            = (homeAngle[id] - lastAngle) / delTime;
        homeAngularAcc[id]       = (homeOmega[id] - lastAngularV) / delTime;
        if(homeAngle[id] > PI)  homeAngle[id] -= 2*PI;
        else if(homeAngle[id] <=-PI) homeAngle[id] += 2*PI;
        homeLastUpdateTime[id]   = timeCapture;
        
		//Adding vl,vr calculation from motion-simulation
		BotPose p1(bsQ.front().first.homePos[id].x, bsQ.front().first.homePos[id].y, bsQ.front().first.homeAngle[id]);
        BotPose p2(newx, newy, newangle);
		homeVlVr[id] = calcBotVelocity(p1, p2, timeMs);
//        if(id == 1)
//          printf("%f %f %f %f\n", homePose[1].x, homePosK[1].x, homePosSigmaSqK[1].x, homeVelocity[1].x);
        checkValidX(homePose[id].x, homeVelocity[id].x, newx);
        checkValidY(homePose[id].y, homeVelocity[id].y, newy);
        checkValidA(homeAngle[id], homeOmega[id], newangle);
    //    printf("homePos from kalman: id: %d, xy: %.0f %.0f\n", id, homePose[id].x, homePose[id].y);
      }
      // Yellow  Away robot info
      uniqueBotIDs.clear();
      for (int i = 0; i < yellowNum; ++i)
      {
        
        SSL_DetectionRobot robot = detection.robots_yellow(i);
				int newx = robot.x() - CENTER_X;
        int newy = robot.y() - CENTER_Y;
        float newangle = robot.orientation();
        #if GR_SIM_COMM || FIRASSL_COMM
        linearTransform(newx, newy, newangle);
        #endif
        #if FIRASSL_COMM
        botcenterTransform(newx, newy, newangle);
        #endif
        //int                id    = HAL::YellowMarkerMap[robot.robot_id()];				
				/* Arpit: Modifying code here to assuming that NO botid of opp team is known. Plz be careful!
				 * Randomly assigning IDs to each bot. if less no. of bots know, then 0...i ids will be populated, 
				 * i+1...4 ids will not be used. 
				 */
				int id_ = getClosestBotID(newx, newy, newangle, uniqueBotIDs);
				if(id_ == -1) //means all bots already populated
					continue;
				int id = HAL::YellowMarkerMap[id_];
				uniqueBotIDs.insert(id);
        //printf("ID = %d, loc = %d, %d\n", id, newx, newy);
        double           delTime = timeCapture - awayLastUpdateTime[id];

        awayPosSigmaSqK[id].x    = awayPosSigmaSqK[id].x * ( 1 - awayPosK[id].x) + SIGMA_SQ_NOISE_POS * delTime;
        awayPosK[id].x           = awayPosSigmaSqK[id].x / (awayPosSigmaSqK[id].x + SIGMA_SQ_OBSVN_POS);
        float  predictedPoseX    = awayPose[id].x + awayVelocity[id].x * (delTime);
        float  lastPoseX         = awayPose[id].x;
        awayPose[id].x           = /*newx;*/predictedPoseX + awayPosK[id].x * (newx - predictedPoseX);
        float lastVelocityx      = awayVelocity[id].x;
        awayVelocity[id].x       = (awayPose[id].x - lastPoseX) / delTime;
        awayAcc[id].x            = (awayVelocity[id].x - lastVelocityx) / delTime;
                                
        awayPosSigmaSqK[id].y    = awayPosSigmaSqK[id].y * ( 1 - awayPosK[id].y) + SIGMA_SQ_NOISE_POS * delTime;
        awayPosK[id].y           = awayPosSigmaSqK[id].y / (awayPosSigmaSqK[id].y + SIGMA_SQ_OBSVN_POS);
        float  predictedPoseY    = /*newy;*/awayPose[id].y + awayVelocity[id].y * (delTime);
        float  lastPoseY         = awayPose[id].y;
        awayPose[id].y           = predictedPoseY + awayPosK[id].y * (newy - predictedPoseY);
        float lastVelocityy      = awayVelocity[id].y;
        awayVelocity[id].y       = (awayPose[id].y - lastPoseY) / delTime;
        awayAcc[id].y            = (awayVelocity[id].y - lastVelocityy) / delTime;
                                
        awayAngleSigmaSqK[id]    = awayAngleSigmaSqK[id] * ( 1 - awayAngleK[id]) + SIGMA_SQ_NOISE_ANGLE * delTime;
        awayAngleK[id]           = awayAngleSigmaSqK[id] / (awayAngleSigmaSqK[id] + SIGMA_SQ_OBSVN_ANGLE);
        float  predictedAngle    = awayAngle[id] + awayOmega[id] * (delTime);
        float  lastAngle         = awayAngle[id];
        awayAngle[id]            = /*nearestAngle(newangle, awayAngle[id]);*/predictedAngle + awayAngleK[id] * (nearestAngle(newangle, awayAngle[id]) - predictedAngle);
        float lastAngularV       = awayOmega[id];
        awayOmega[id]            = (awayAngle[id] - lastAngle) / delTime;
        awayAngularAcc[id]       = (awayOmega[id] - lastAngularV) / delTime;
        if(awayAngle[id] > PI)        awayAngle[id] -= 2*PI;
        else if(awayAngle[id] <=-PI)  awayAngle[id] += 2*PI;
        awayLastUpdateTime[id]   = timeCapture;
		
		//Adding vl,vr calculation from motion-simulation
		BotPose p1(bsQ.front().first.awayPos[id].x, bsQ.front().first.awayPos[id].y, bsQ.front().first.awayAngle[id]);
        BotPose p2(newx, newy, newangle);
		homeVlVr[id] = calcBotVelocity(p1, p2, timeMs);
		
        checkValidX(awayPose[id].x, awayVelocity[id].x, newx);
        checkValidY(awayPose[id].y, awayVelocity[id].y, newy);
        checkValidA(awayAngle[id], awayOmega[id], newangle);
        //printf("Awaybots %d: %f %f %f %f %lf\n", id, awayPose[id].x, awayPose[id].y, predictedPoseX, awayPosK[id].y, delTime);
      }
    }
    else
    {
      // Home Yellow robot info
      uniqueBotIDs.clear();
      for (int i = 0; i < yellowNum; ++i)
      {
        SSL_DetectionRobot robot = detection.robots_yellow(i);
        int id    = HAL::YellowMarkerMap[robot.robot_id()];
		//printf("Kalman:: ID%d ",id);
		//assert(false);
        if(uniqueBotIDs.find(id) != uniqueBotIDs.end())
          continue;
        uniqueBotIDs.insert(id);
        int newx = robot.x() - CENTER_X;
        int newy = robot.y() - CENTER_Y;        
        float newangle = robot.orientation();
        #if GR_SIM_COMM || FIRASSL_COMM
        linearTransform(newx, newy, newangle);
        #endif
        #if FIRASSL_COMM
        botcenterTransform(newx, newy, newangle);
        #endif
        double           delTime = timeCapture - homeLastUpdateTime[id];
        homePosSigmaSqK[id].x    = homePosSigmaSqK[id].x * ( 1 - homePosK[id].x) + SIGMA_SQ_NOISE_POS * delTime;
        homePosK[id].x           = homePosSigmaSqK[id].x / (homePosSigmaSqK[id].x + SIGMA_SQ_OBSVN_POS);
        float  predictedPoseX    = homePose[id].x + homeVelocity[id].x * (delTime);
        float  lastPoseX         = homePose[id].x;
        homePose[id].x           = predictedPoseX + homePosK[id].x * (newx - predictedPoseX);
        float lastVelocityx      = homeVelocity[id].x;
        homeVelocity[id].x       = (homePose[id].x - lastPoseX) / delTime;
        homeAcc[id].x            = (homeVelocity[id].x - lastVelocityx) / delTime;
                                
        homePosSigmaSqK[id].y    = homePosSigmaSqK[id].y * ( 1 - homePosK[id].y) + SIGMA_SQ_NOISE_POS * delTime;
        homePosK[id].y           = homePosSigmaSqK[id].y / (homePosSigmaSqK[id].y + SIGMA_SQ_OBSVN_POS);
        float  predictedPoseY    = homePose[id].y + homeVelocity[id].y * (delTime);
        float  lastPoseY         = homePose[id].y;
        homePose[id].y           = predictedPoseY + homePosK[id].y * (newy - predictedPoseY);
		//assert(homePose[id].x!=NULL);
		
        float lastVelocityy      = homeVelocity[id].y;
        homeVelocity[id].y       = (homePose[id].y - lastPoseY) / delTime;
        homeAcc[id].y            = (homeVelocity[id].y - lastVelocityy) / delTime;
                                
        homeAngleSigmaSqK[id]    = homeAngleSigmaSqK[id] * ( 1 - homeAngleK[id]) + SIGMA_SQ_NOISE_ANGLE * delTime;
        homeAngleK[id]           = homeAngleSigmaSqK[id] / (homeAngleSigmaSqK[id] + SIGMA_SQ_OBSVN_ANGLE);
        float  predictedAngle    = homeAngle[id] + homeOmega[id] * (delTime);
        float  lastAngle         = homeAngle[id];
        homeAngle[id]            = predictedAngle + homeAngleK[id] * (nearestAngle(newangle, homeAngle[id]) - predictedAngle);
        float lastAngularV       = homeOmega[id];
        homeOmega[id]            = (homeAngle[id] - lastAngle) / delTime;
        homeAngularAcc[id]       = (homeOmega[id] - lastAngularV) / delTime;
        if(homeAngle[id] > PI)  homeAngle[id] -= 2*PI;
        else if(homeAngle[id] <=-PI) homeAngle[id] += 2*PI;
        checkValidX(homePose[id].x, homeVelocity[id].x, newx);
        checkValidY(homePose[id].y, homeVelocity[id].y, newy);
        checkValidA(homeAngle[id], homeOmega[id], newangle);
        homeLastUpdateTime[id]   = timeCapture;
		
		//Adding vl,vr calculation from motion-simulation
		BotPose p1(bsQ.front().first.homePos[id].x, bsQ.front().first.homePos[id].y, bsQ.front().first.homeAngle[id]);
        BotPose p2(newx, newy, newangle);
		homeVlVr[id] = calcBotVelocity(p1, p2, timeMs);
      }
	  
      // Blue robot info
      uniqueBotIDs.clear();
      for (int i = 0; i < blueNum; ++i)
      {
        
        SSL_DetectionRobot robot = detection.robots_blue(i);        
        int newx = robot.x() - CENTER_X;
        int newy = robot.y() - CENTER_Y;
        float newangle = robot.orientation();
        #if GR_SIM_COMM || FIRASSL_COMM
        linearTransform(newx, newy, newangle);
        #endif
        #if FIRASSL_COMM
        botcenterTransform(newx, newy, newangle);
        #endif						
				/* Arpit: Modifying code here to assuming that NO botid of opp team is known. Plz be careful!
				 * Randomly assigning IDs to each bot. if less no. of bots know, then 0...i ids will be populated, 
				 * i+1...4 ids will not be used. 
				 */
				int id_ = getClosestBotID(newx, newy, newangle, uniqueBotIDs);
				if(id_ == -1) //means all bots already populated
					continue;
				int id = HAL::BlueMarkerMap[id_];
				uniqueBotIDs.insert(id);
        //printf("ID = %d, loc = %d, %d\n", id, newx, newy);
				
				
        double           delTime = timeCapture - awayLastUpdateTime[id];

        awayPosSigmaSqK[id].x    = awayPosSigmaSqK[id].x * ( 1 - awayPosK[id].x) + SIGMA_SQ_NOISE_POS * delTime;
        awayPosK[id].x           = awayPosSigmaSqK[id].x / (awayPosSigmaSqK[id].x + SIGMA_SQ_OBSVN_POS);
        float  predictedPoseX    = awayPose[id].x + awayVelocity[id].x * (delTime);
        float  lastPoseX         = awayPose[id].x;
        awayPose[id].x           = /*newx;*/predictedPoseX + awayPosK[id].x * (newx - predictedPoseX);
        float lastVelocityx      = awayVelocity[id].x;
        awayVelocity[id].x       = (awayPose[id].x - lastPoseX) / delTime;
        awayAcc[id].x            = (awayVelocity[id].x - lastVelocityx) / delTime;
                                
        awayPosSigmaSqK[id].y    = awayPosSigmaSqK[id].y * ( 1 - awayPosK[id].y) + SIGMA_SQ_NOISE_POS * delTime;
        awayPosK[id].y           = awayPosSigmaSqK[id].y / (awayPosSigmaSqK[id].y + SIGMA_SQ_OBSVN_POS);
        float  predictedPoseY    = awayPose[id].y + awayVelocity[id].y * (delTime);//newy;
        float  lastPoseY         = awayPose[id].y;
        awayPose[id].y           = predictedPoseY + awayPosK[id].y * (newy - predictedPoseY);
        float lastVelocityy      = awayVelocity[id].y;
        awayVelocity[id].y       = (awayPose[id].y - lastPoseY) / delTime;
        awayAcc[id].y            = (awayVelocity[id].y - lastVelocityy) / delTime;
                                
        awayAngleSigmaSqK[id]    = awayAngleSigmaSqK[id] * ( 1 - awayAngleK[id]) + SIGMA_SQ_NOISE_ANGLE * delTime;
        awayAngleK[id]           = awayAngleSigmaSqK[id] / (awayAngleSigmaSqK[id] + SIGMA_SQ_OBSVN_ANGLE);
        float  predictedAngle    = awayAngle[id] + awayOmega[id] * (delTime);
        float  lastAngle         = awayAngle[id];
        awayAngle[id]            = /*nearestAngle(newangle, awayAngle[id]);*/predictedAngle + awayAngleK[id] * (nearestAngle(newangle, awayAngle[id]) - predictedAngle);
        float lastAngularV       = awayOmega[id];
        awayOmega[id]            = (awayAngle[id] - lastAngle) / delTime;
        awayAngularAcc[id]       = (awayOmega[id] - lastAngularV) / delTime;
        if(awayAngle[id] > PI)        awayAngle[id] -= 2*PI;
        else if(awayAngle[id] <=-PI)  awayAngle[id] += 2*PI;
        checkValidX(awayPose[id].x, awayVelocity[id].x, newx);
        checkValidY(awayPose[id].y, awayVelocity[id].y, newy);
        checkValidA(awayAngle[id], awayOmega[id], newangle);
        
        awayLastUpdateTime[id]   = timeCapture;
        //printf("Awaybots %d: %f %f %f %f %lf\n", id, awayPose[id].x, awayPose[id].y, predictedPoseX, awayPosK[id].y, delTime);
		//Adding vl,vr calculation from motion-simulation
		BotPose p1(bsQ.front().first.awayPos[id].x, bsQ.front().first.awayPos[id].y, bsQ.front().first.awayAngle[id]);
        BotPose p2(newx, newy, newangle);
		homeVlVr[id] = calcBotVelocity(p1, p2, timeMs);
      }
    }

	bsQ.pop();
	bsQ.push(std::make_pair(BeliefState(), nowTime));

    mutex->leave();
  }

  void Kalman::update(BeliefState& state)
  {
    mutex->enter();
    // TODO : Add delTime here and update the state state with respect to delTime
    struct timeval tv2;
    gettimeofday(&tv2, NULL);
    double timeNow      = tv2.tv_sec+tv2.tv_usec/1000000.0;
    state.currFrameNum++;
	
	/* new adds for ourNum and oppNum*/
    if (HomeTeam::COLOR == Simulator::BLUE_TEAM)
    {
      state.ourNum = blueNum;
      state.oppNum = yellowNum;
    }
    else if(HomeTeam::COLOR == Simulator::YELLOW_TEAM)
    {
      state.ourNum = yellowNum;
      state.oppNum = blueNum;
    }
    else
    {
      assert(HomeTeam::COLOR == Simulator::YELLOW_TEAM || HomeTeam::COLOR == Simulator::BLUE_TEAM);
    }
	
    for (int botID = 0; botID < HomeTeam::SIZE; ++botID)
    {
      double delTime = 0;
      state.homePos[botID]   = Vector2D<int>(homePose[botID].x, homePose[botID].y);/*Vector2D<int>(homePose[botID].x+homeVelocity[botID].x*delTime, 
      homePose[botID].y+homeVelocity[botID].y*delTime);*/
      printf("Home Pose: %d %d %d\n",botID,state.homePos[botID].x,state.homePos[botID].y);
	  state.homeAngle[botID] = homeAngle[botID];// + homeOmega[botID]*delTime;
      state.homeVel[botID]   = homeVelocity[botID];
	  state.homeVlVr[botID] = homeVlVr[botID];
      state.homeOmega[botID] = homeOmega[botID];
      state.homeAcc[botID]   = homeAcc[botID];
      state.homeAngAcc[botID]= homeAngularAcc[botID];
    }
    //printf("updating %d %d\n", state.homePos[1].x, state.homePos[1].y);
    for (int botID = 0; botID < AwayTeam::SIZE; ++botID)
    {
      double delTime = 0;
      state.awayPos[botID]   = Vector2D<int>(awayPose[botID].x, awayPose[botID].y);
      state.awayAngle[botID] = awayAngle[botID];
      state.awayVel[botID]   = awayVelocity[botID];
      state.awayOmega[botID] = awayOmega[botID];
      state.awayAcc[botID]   = awayAcc[botID];
      state.awayAngAcc[botID]= awayAngularAcc[botID];
    }
      double delTime = 0;
	  
      state.ballPos = Vector2D<int>(ballPose.x + delTime*ballVelocity.x, ballPose.y + delTime*ballVelocity.y);
      state.ballVel = ballVelocity;
      state.ballAcc = ballAcceleration;
    //printf("Omega-> %lf\n",state.homeOmega[2]);

        /*for (int botID = 0; botID < HomeTeam::SIZE; ++botID)
    {
        if((v.readBuffer.botArr[HOME_TEAM][botID].center.x != 0) && (v.readBuffer.botArr[HOME_TEAM][botID].center.y != 0))
        {
      state.homePos[botID]   = Vector2D<int>(v.readBuffer.botArr[HOME_TEAM][botID].center.x,v.readBuffer.botArr[HOME_TEAM][botID].center.y);
      
      //state.homeAngle[botID] = homeAngle[botID];
          state.homeAngle[botID] = v.readBuffer.botArr[HOME_TEAM][botID].orientation;
        }
      //state.homeVel[botID]   = homeVelocity[botID];
      //state.homeOmega[botID] = homeOmega[botID];
                        //state.homeAcc[botID]   = homeAcc[botID];
                        //state.homeAngAcc[botID]= homeAngularAcc[botID];
    }
        for (int botID = 0; botID < AwayTeam::SIZE; ++botID)
    {
      state.awayPos[botID]   = Vector2D<int>(v.readBuffer.botArr[AWAY_TEAM][botID].center.x,v.readBuffer.botArr[AWAY_TEAM][botID].center.y);
      //state.homeAngle[botID] = homeAngle[botID];
      state.awayAngle[botID] = v.readBuffer.botArr[AWAY_TEAM][botID].orientation;
      //state.homeVel[botID]   = homeVelocity[botID];
      //state.homeOmega[botID] = homeOmega[botID];
                        //state.homeAcc[botID]   = homeAcc[botID];
                        //state.homeAngAcc[botID]= homeAngularAcc[botID];
    }
    state.ballPos = Vector2D<int>(v.readBuffer.ball.x,v.readBuffer.ball.y);
    printf("Ball pos %d,%d\n",state.ballPos.x,state.ballPos.y);
    printf("Aqua bot %d,%d\n",state.homePos[1].x,state.homePos[1].x);*/
    
    state.computeBallLocation();
    state.computeBallInDBox();
    //state.computeBallSide();
    state.computeBallInStrips();
    mutex->leave();
  }
}