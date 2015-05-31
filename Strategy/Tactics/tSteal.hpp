#ifndef TSTEAL_HPP
#define TSTEAL_HPP

#include <list>
#include "comdef.h"
#include "tactic.h"
#include "skillSet.h"
#include "beliefState.h"
#include "logger.h"
#include "config.h"
#define D_STEAL 0
namespace Strategy
{
  class TSteal : public Tactic
  {
  public:
    TSteal(const BeliefState* state, int botID) : Tactic(Tactic::Stop, state, botID)
    { } // TSteal

    ~TSteal()
    { } // ~TSteal

    inline bool isActiveTactic(void) const
    {
      //return true;
      return true;
    }

    int chooseBestBot(std::list<int>& freeBots, const Tactic::Param* tParam, int prevID) const
    {
      int minv = *(freeBots.begin());
      int mindis = 1000000000;
      for(std::list<int>::iterator it = freeBots.begin(); it != freeBots.end(); ++it)
      {
        float dis_from_opp = (state->homePos[*it] - state->awayPos[state->oppBotNearestToBall]).absSq();
        if(*it == botID)
          dis_from_opp -= HYSTERESIS;
        if(dis_from_opp < mindis)
        {
          mindis = dis_from_opp;
          minv = *it;
        }
      }
      printf("%d assigned Steal\n", minv);
      return minv;
    } // chooseBestBot

    void execute(const Param& tParam)
    {
      printf("Steal BotID: %d\n",botID);
      
      if(state->pr_looseBall == true )
      {
        float angle_to_ball = Vector2D<int>::angle(state->ballPos, state->homePos[botID]);
        captureBall(angle_to_ball);
        // Util::Logger::toStdOut("Going to  : %f,%f\n",sParam.GoToPointP.x,sParam.GoToPointP.y);
      }
      if(state->pr_oppBall == true)
      {
        float angle_to_ball = Vector2D<int>::angle(state->ballPos, state->homePos[botID]);
        captureBall(angle_to_ball);
      }
      if(state->pr_ourBall == true) // our team takes possession
      {
        debug(D_STEAL, "OUR BALL");
        if(state->ourBotNearestToBall == botID)
          standAndDribble();
        else
          coverBall();
        tState = COMPLETED;
        debug(D_STEAL, "Steal Completed!");
      }
    }
  }; // class TSteal
} // namespace Strategy

#endif // TSTEAL_HPP
