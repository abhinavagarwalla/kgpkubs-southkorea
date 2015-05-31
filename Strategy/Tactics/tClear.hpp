#ifndef TTCLEAR_HPP
#define TTCLEAR_HPP

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
  class TClear : public Tactic
  {
  public:
    TClear(const BeliefState* state, int botID) :
      Tactic(Tactic::Stop, state, botID)
    { 
    } // TCharge

    ~TCharge()
    { } // ~TCharge

    inline bool isActiveTactic(void) const
    {
      return true;
    }

    int chooseBestBot(std::list<int>& freeBots, const Tactic::Param* tParam, int prevID) const
    {
      int minv = *(freeBots.begin());
      float minanglediff = Vector2D<float>::angle(state->homePos[minv],state->ballPos);
      for (std::list<int>::iterator it = freeBots.begin(); it != freeBots.end(); ++it)
      {
        // TODO make the bot choosing process more sophisticated, the logic below returns the 1st available bot
        if(ForwardX(state->homePos[*it].x) < ForwardX(state->ballPos.x) && Vector2D<int>::dist(state->homePos[*it], state->ballPos) < mindis)
        {
          float currangle = Vector2D<float>::angle(state->homePos[*it], state->ballPos);
					float anglediff = state->homeAngle[*it] - currangle;
					if(minanglediff < anglediff)
					{
						minanglediff = anglediff;
						minv = *it;
					}
        }
      }
      return minv;
    } // chooseBestBot


    void execute(const Param& tParam)
    {
      printf("Clear BotID: %d\n",botID);

        sID = SkillSet::Velocity;
        sParam.VelocityP.vl = MAX_BOT_SPEED;
				sParam.VelocityP.vr = MAX_BOT_SPEED;
		skillSet->executeSkill(sID, sParam);
    
    float distx = fabs(state->ballPos.x-OPP_GOAL_X);
    }
  };
// class TCharge
} // namespace Strategy

#endif // TTCharge_HPP
