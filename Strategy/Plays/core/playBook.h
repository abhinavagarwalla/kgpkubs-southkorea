#ifndef PLAY_BOOK_H
#define PLAY_BOOK_H
#include <utility>
//
#include <list>
#include "comdef.h"
#include "tactic.h"
#include "skillSet.h"
#include "beliefState.h"
#include "logger.h"
#include "config.h"
//
// Forward Declarations
namespace Strategy
{
  class BeliefState;
  class Play;
}

namespace Strategy
{
  class PlayBook
  {
  public:
    enum PlayID
    {
       Stop =0,
    PositionGather,
    Offense1,
    Offense2  ,
    Defense1  ,
    Defense2  ,
    PositionOurKickoff  ,
    PositionOppKickoff  ,
    PositionOurFreeKick  ,
    PositionOppFreeKick  ,
    PositionOurFreeBall  ,
    PositionOppFreeBall  ,
    PositionOurPenalty  ,
    PositionOppPenalty  ,
    PositionOurGoalKick  ,
    PositionOppGoalKick  ,
    Kickoff  ,
    OurFreeKick  ,
    OppFreeKick  ,
    OurFreeBall  ,
    OppFreeBall  ,
    PenaltyOur  ,
    PenaltyOpp  ,
    OurGoalKick  ,
    OppGoalKick  ,
    TakeGoalKick  ,
    TestPlay  ,
    SetPosition  ,
    SuperOffense  ,
    SuperDefense  ,
    None  ,
      MAX_PLAYS //30 should be it. 31, it is.
    };

  protected:
    const BeliefState* state;
    Play*              playList[MAX_PLAYS];

  public:
    PlayBook(const BeliefState* state);
    
    
    ~PlayBook();

    void reload(void);
  }; // class PlayBook
} // namespace Strategy

#endif // PLAY_BOOK_H