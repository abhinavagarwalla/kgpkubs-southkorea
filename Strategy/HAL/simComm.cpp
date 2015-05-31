#include "simComm.h"
#include "cs.hpp"
#include "shmem.h"
#include "thread.h"
#include "logger.h"

using namespace Util;

namespace HAL
{
  SimComm::SimComm()
  {
    switch (Strategy::HomeTeam::COLOR)
    {
      case Simulator::BLUE_TEAM:
        shmem = new Shmem(SHM_BLUE_ALIAS, sizeof(SimPacket));
        break;

      case Simulator::YELLOW_TEAM:
        shmem = new Shmem(SHM_YELLOW_ALIAS, sizeof(SimPacket));
        break;

      default:
        Logger::abort("Invalid Team Color");
    }

    // Resetting all the commands in the shared memory
    sendDebugInfo(NULL, 0);
    for (int i = 0; i < Strategy::HomeTeam::SIZE; ++i)
    {
      sendCommand(i, 0, 0, 0, 0, false);
    }
  }

  SimComm::~SimComm()
  {
    delete shmem;
    shmem = NULL;
  }

  void SimComm::sendDebugInfo(const CanvasItem* items, int numItems)
  {
    commCS.enter();
    for (int i = 0; i < numItems; ++i)
    {
      command.items[i] = items[i];
    }
    command.numItems = numItems;

    shmem->write(&command, sizeof(SimPacket));
    commCS.leave();
  }

  void SimComm::sendCommand(int   botID,
                            float v_x,
                            float v_y,
                            float v_t,
                            float kickPower,
                            bool  dribble)
  {
    commCS.enter();
    command.bot[botID].v_x       = v_x;
    command.bot[botID].v_y       = v_y;
    command.bot[botID].v_t       = v_t;
    command.bot[botID].kickPower = kickPower;
    command.bot[botID].dribble   = dribble;

    shmem->write(&command, sizeof(SimPacket));
    commCS.leave();
  }
} // namespace HAL
