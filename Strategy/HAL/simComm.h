/* This header file contains the definition of
 * class SimComm  which inherits Comm class.
 * This class is used to  assign commands to the all
 * the bots of the team - v_x, v_y, v_t, kickpower, dribble
 */

#ifndef SIM_COMM_H
#define SIM_COMM_H

#include "comm.h"
#include "rbCommDef.h"

// Forward Declarations
namespace Util
{
  class Shmem;
  class CS;
}

namespace HAL
{
  class SimComm : public Comm
  {
  private:
    Util::Shmem* shmem;
    SimPacket    command;

  public:
    SimComm();

    ~SimComm();

    void sendDebugInfo(const CanvasItem* items, int numItems);

    void sendCommand(int   botID,
                     float v_x,
                     float v_y,
                     float v_t,
                     float kickPower,
                     bool  dribble);
  };
}

#endif // SIM_COMM_H
