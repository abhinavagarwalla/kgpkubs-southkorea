#include <math.h>
#include "pose.h"
#include "velocity-profile.hpp"
#include "trajectory.hpp"
#include "splines.hpp"
#include "geometry.hpp"
#include "trajectory-generators.cpp"
#include <iostream>

namespace BallInterception {


inline Vector2D<float> predictBallPose(Vector2D<float> ballPos, Vector2D<float> ballVel, double timeOfPrediction){
    Vector2D<float> bPos;
    bPos.x = ballPos.x + timeOfPrediction*ballVel.x;
    bPos.y = ballPos.y + timeOfPrediction*ballVel.y;

    while (bPos.x > HALF_FIELD_MAXX || bPos.x < -HALF_FIELD_MAXX || bPos.y > HALF_FIELD_MAXY || bPos.y < -HALF_FIELD_MAXY) {
        if (bPos.y > HALF_FIELD_MAXY) {
            bPos.y = 2 * HALF_FIELD_MAXY - bPos.y;
        }
        if (bPos.y < -HALF_FIELD_MAXY) {
            bPos.y = - (2 * HALF_FIELD_MAXY + bPos.y);
        }
        if (bPos.x > HALF_FIELD_MAXX) {
            bPos.x = 2 * HALF_FIELD_MAXX - bPos.x;
        }
        if (bPos.x < -HALF_FIELD_MAXX) {
            bPos.x = - (2 * HALF_FIELD_MAXX + bPos.x);
        }
    }

    return bPos;
}

inline double getBotBallDist(Pose botPos, Vector2D<float> ballPos) {
    return sqrt((botPos.x() - ballPos.x)*(botPos.x() - ballPos.x) + (botPos.y() - ballPos.y)*(botPos.y() - ballPos.y));
}

inline SplineTrajectory* getIntTraj(Pose botPosStart, Vector2D<float> ballPos, Vector2D<float> ballVel, Vector2D<float> botVel) {

    Vector2D<float> predictedBallPos;
    double error = 0.01;
    double T2 = 6.0;
    double T1 = 0.0;
    double S = 1.0;
    Vector2D<float> goalCentre(ForwardX(HALF_FIELD_MAXX - GOAL_DEPTH), 0);
    SplineTrajectory *st = NULL;
    double T = T1;
    while (1) {
        if (st)
            delete st;
        predictedBallPos = predictBallPose(ballPos, ballVel, T);
        double endTheta = atan2(goalCentre.y - predictedBallPos.y, goalCentre.x - predictedBallPos.x);
        Pose endPose(predictedBallPos.x, predictedBallPos.y, endTheta);
        // add a cp behind the ball pos, distance of 500
      //  Pose cp1(predictedBallPos.x+500*cos(endTheta+M_PI), predictedBallPos.y+500*sin(endTheta+M_PI), 0);
        vector<Pose> midPoints;
       // midPoints.push_back(cp1);
       //st = TrajectoryGenerators::cubic(botPosStart, endPose, 0,0, 40,40 , midPoints);
       st = TrajectoryGenerators::cubic(botPosStart, endPose, botVel.x, botVel.y, 0, 0, midPoints);
        double t = st->totalTime();
        if (t < T)
            break;
        T += S;
        if (T >= 6.0)
            break;
    }
    if (T <= 6.0) {
        T2 = T;
        T1 = T - S;
    }
    //qDebug() << "here3" << endl;
    int iter = 0;
	Pose endPose;
    while (1) {
        // predictedBallPos: strategy coordinates
        double mid = (T1+T2)/2;
        predictedBallPos = predictBallPose(ballPos, ballVel, mid);
        iter++;
        double endTheta = atan2(goalCentre.y - predictedBallPos.y, goalCentre.x - predictedBallPos.x);
        endPose = Pose(predictedBallPos.x, predictedBallPos.y, endTheta);
        if (st)
            delete st;
        // add a cp behind the ball pos, distance of 500
      //  Pose cp1(predictedBallPos.x+500*cos(endTheta+M_PI), predictedBallPos.y+500*sin(endTheta+M_PI), 0);
		vector<Pose> midPoints;
       // midPoints.push_back(cp1);
    //   st = TrajectoryGenerators::cubic(botPosStart, endPose, 0,0, 40, 40, midPoints);
	     st = TrajectoryGenerators::cubic(botPosStart, endPose, botVel.x, botVel.y, 0, 0, midPoints);

        double t = st->totalTime();
		//std::cout << "time" << t << std::endl;
		//getchar();
     //   qDebug() << "mid = " << mid << ", bot-ka-time = " << t;
        if (fabs(t-mid) < error)
            break;
        if (t > mid) {
            T1 = mid;
        } else if (t < mid) {
            T2 = mid;
        }
        if (fabs(T2-T1) < error) {
        //    qDebug() << "T2, T1 almost same = " << T1 <<", t = " << t;
            break;
        }
    }
    return st;
}
}
