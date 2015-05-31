#pragma once
#ifndef SSL_COMM_H
#define SSL_COMM_H
#include "comm.h"
#include "serial.h"
#include "rbCommDef.h"
namespace Util
{
  class CS;
}
namespace HAL
{
  class SSLComm : public Comm
  {
  private:
    Serial    sPort;
    SSLPacket command;
  public:
    SSLComm();
    ~SSLComm();
    void sendCommand(int   botID,
                     float v_x,
                     float v_y,
                     float v_t,
                     float kickPower,
                     bool  dribble);
  }; // class SSLComm
} // namespace HAL
#endif // SSL_COMM_H
