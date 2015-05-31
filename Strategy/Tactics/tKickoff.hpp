 #pragma once

#include <list>
#include "comdef.h"
#include "tactic.h"
#include "skillSet.h"
#include "beliefState.h"
#include "logger.h"
#include "config.h"

namespace Strategy
{
  class TKickoff : public Tactic
  {
		int prevVelocity;
  public:
    TKickoff(const BeliefState* state, int botID) :
      Tactic(Tactic::Stop, state, botID)
    { } // TTStop

    ~TKickoff()
    { } // ~TTStop
    enum InternalState
    {
      APPROACHING,
      SPINNING_CCW = -1,
      SPINNING_CW = 1,
      ATTACKING
    } iState;

    inline bool isActiveTactic(void) const
    {
      return true;
    }

    int chooseBestBot(std::list<int>& freeBots, const Tactic::Param* tParam, int prevID) const
    {
			/*Arpit: tactic explicitly for Kickoff. chooses the farthest bot on the other side of the field. */
			// no. closest to ball.
      int maxv = *(freeBots.begin());
      int maxdis = 0;
      for (std::list<int>::iterator it = freeBots.begin(); it != freeBots.end(); ++it)
      {        
        float dist = (state->homePos[*it].x);
        if(dist > maxdis)
        {
          maxdis= dist;
          maxv = *it;
        }
      }
	  printf("Kick off is working \n");
      //debug(1, maxv, "assigned Pass");
	  printf(" :: %d ::",maxv);
      // assert(tParam=0);
      return maxv;
    } // chooseBestBot

    void execute(const Param& tParam)
    {			
			if(tState == Tactic::INIT)
			{
				printf("prev velocity init .................................***********\n");
				tState = Tactic::RUNNING;
				prevVelocity = 5;
			}				
      printf("TKickoff BotID: %d\n",botID);
      float angle = state->homeAngle[botID];
			int sign = 1;
			//assert(angle >= -PI && angle <= PI);
			if((angle < PI / 2 || angle > -PI/2))
			{
				if(ForwardX(1) > 0)
					sign = 1;
				else
					sign = -1;
			} 
			else {
				if(ForwardX(1) > 0)
					sign = 1;
				else
					sign = -1;
			}
			printf("the bot id for kickoff issss  : %d \n",botID);
			
			float yDiff = abs(state->ballPos.y - state->homePos[botID].y);
			printf("    the ydiffffffffffffffffffffffffffff  is :: %f \n",yDiff);
			
			if(yDiff > 4*BOT_RADIUS) //240
				return;
			prevVelocity += 5;
			if(prevVelocity > 120)
				prevVelocity = 120;
			sign = 1;
			sID = SkillSet::Velocity;
			sParam.VelocityP.vl = prevVelocity*sign;
			sParam.VelocityP.vr = prevVelocity*sign;
			skillSet->executeSkill(sID, sParam);			
    }
  }; // class TKickoff
} // namespace Strategy


