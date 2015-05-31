#include "sslComm.h"
#include "serial.h"
#include "cs.hpp"
#include "logger.h"
#include "rbCommDef.h"
using namespace std;
using namespace Util;

namespace HAL
{
  SSLComm::SSLComm()
  {
    if (!sPort.Open("/dev/ttyUSB0", 38400))
    {
      Logger::abort("Could not open COM port");
    }
  }

  SSLComm::~SSLComm()
  { }

  void SSLComm::sendCommand(int   botID,
                            float v_x,
                            float v_y,
                            float v_t,
                            float kickPower,
                            bool  dribble)
  {
    commCS.enter();
    int pwmW1         = (int)((v_t - v_x) * 100.0f / 50);
    int pwmW2         = (int)(((PI / 2 * v_y / 2) + (v_x / 2) + v_t) * 100.0f / 50);
    int pwmW0         = (int)(((-PI / 2 * v_y / 2) + (v_x / 2) + v_t) * 100.0f / 50);

    command.preamble  = 0xAA;
    command.teamColor = (uint8_t)Strategy::HomeTeam::COLOR;
    command.botID     = botID;
    command.dirW0     = pwmW0 > 0 ? 1 : 0;
    command.dirW2     = pwmW2 > 0 ? 1 : 0;
    command.dirW1     = pwmW1 > 0 ? 1 : 0;
    command.dribble   = dribble ? 1 : 0;
    command.kickPower = (kickPower > 0.1)?3:0;

    if (pwmW1 < 0)
    {
      pwmW1 = -pwmW1;
    }
    if (pwmW2 < 0)
    {
      pwmW2 = -pwmW2;
    }
    if (pwmW0 < 0)
    {
      pwmW0 = -pwmW0;
    }

    command.pwmW0     = pwmW0;
    command.pwmW2     = pwmW2;
    command.pwmW1     = pwmW1;

    sPort.Write(&command, sizeof(SSLPacket));
    commCS.leave();
  }
} // namespace HAL
