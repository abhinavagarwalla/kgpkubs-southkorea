#ifndef TTDRAGTOGOAL_HPP
#define TTDRAGTOGOAL_HPP


#include <list>
#include "comdef.h"
#include "tactic.h"
#include "skillSet.h"
#include "beliefState.h"
#include "logger.h"
#include "config.h"

//TODO: Make a tactic such as tStealAndShoot which steals the ball, then shoots it, and
//      continues the stealing a few times if a goal isn't scored.

namespace Strategy
{
  class TDragToGoal : public Tactic
  {
  public:
    TDragToGoal(const BeliefState* state, int botID) :
      Tactic(Tactic::Stop, state, botID)
    { } // TCharge

    ~TDragToGoal()
    { } // ~TCharge

    inline bool isActiveTactic(void) const
    {
      return true;
    }

    int chooseBestBot(std::list<int>& freeBots, const Tactic::Param* tParam, int prevID) const
    {
      int minv = *(freeBots.begin());
      int mindis = 1000000000;
      for (std::list<int>::iterator it = freeBots.begin(); it != freeBots.end(); ++it)
      {
        // TODO make the bot choosing process more sophisticated, the logic below returns the 1st available bot
        float dis_from_ball = (state->homePos[*it] - state->ballPos).absSq();
        if(dis_from_ball < mindis)
        {
          mindis = dis_from_ball;
          minv = *it;
        }
      }
      if(mindis > 2 * BOT_POINT_THRESH)
      {
        for (std::list<int>::iterator it = freeBots.begin(); it != freeBots.end(); ++it)
        {
          int bot_id = *it;
          if(isBallInMyWideAngleRange(bot_id))
          {
            minv = *it;
            break;
          }
        }
      }
      //Util::Logger::toStdOut("Selected bot %d\n", minv);
      printf("%d assigned DragToGoal\n", minv);
      return minv;

    } // chooseBestBot

    void execute(const Param& tParam)
    {
      printf("DragtoGoal BotID: %d\n",botID);
      
      static int stt = 0;
      // Select the skill to be executed next
      Vector2D<int> fastBallPos;
      Vector2D<float> origin(0, 0);
      fastBallPos  = Vector2D<int>((int)state->ballPos.x + 0.3 * state->ballVel.x, (int)state->ballPos.y + 0.3 * state->ballVel.y);
      if(state->ballVel.absSq() > MOVING_BALL_VELOCITY * MOVING_BALL_VELOCITY)
      {
        Util::Logger::toStdOut("Capturing fast Ball %f\n", state->ballVel.absSq());
        float captureDist = Vector2D<int>::dist(state->ballPos, state->homePos[botID]);
//        float captureAngle = fabs(firaNormalizeAngle(Vector2D<float>::angle(state->ballVel, origin)-Vector2D<int>::angle(state->homePos[botID], state->ballPos)));
//        if(captureAngle < PI/3) {
//          Util::Logger::toStdOut("Ball now in range\n");
//          fastBallPos  = Vector2D<int>((int)state->ballPos.x + 0.1*state->ballVel.x, (int)state->ballPos.y + 0.1*state->ballVel.y);
//        }
      }
      Vector2D<int> goalPoint(HALF_FIELD_MAXX, 0);
      float theta               = normalizeAngle(Vector2D<int>::angle(goalPoint, fastBallPos));
      float angleWithBall       = normalizeAngle(Vector2D<int>::angle(fastBallPos, state->homePos[botID]));
      //Vector2D<int> finalPoint  = state->homePos[botID] + Vector2D<int>(2*BOT_BALL_THRESH*cos(theta), 2*BOT_BALL_THRESH*sin(theta));

      Vector2D<int> targetPoint = fastBallPos - Vector2D<int>(2 * BOT_BALL_THRESH * cos(theta), 2 * BOT_BALL_THRESH * sin(theta));
      float dist = Vector2D<int>::dist(targetPoint, state->homePos[botID]);
      //printf("Drag to Goal: %f, %d, %f, \n", dist, 2*BOT_BALL_THRESH, fabs(firaNormalizeAngle(theta)));
      float sign = (state->homePos[botID].x - fastBallPos.x) * (state->homePos[botID].x - HALF_FIELD_MAXX);
      printf("state: %d\n", stt);
      if(stt == 0 && isBallInMyWideAngleRange() && sign > 0)
      {
        stt = 1;
      }
      if(stt == 1 && (!isBallInMyLastHopeRange(botID) || sign < 0))
      {
        stt = 0;
      }
      if(stt == 1)
      {
        Util::Logger::toStdOut("Dribbling to goal %d %d", goalPoint.x, goalPoint.y);
        gotoPointExact(state->ballPos.x, state->ballPos.y, true, normalizeAngle(Vector2D<int>::angle( goalPoint, fastBallPos )), MAX_BOT_SPEED);
      }
      else
      {
        //Util::Logger::toStdOut("Going to ball");
        gotoPointExact(targetPoint.x, targetPoint.y, true, Vector2D<int>::angle(goalPoint, state->homePos[botID]), 0.7*MAX_BOT_SPEED);
      }
      if (state->pr_oppBall || state->pr_looseBall/*||state->pr_goalscored*/)
      {
//      tState = COMPLETED;
      }
    }
  }; // class TCharge
} // namespace Strategy

#endif // TTCharge_HPP
