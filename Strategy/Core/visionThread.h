#pragma once
#ifndef VISION_THREAD_H
#define VISION_THREAD_H

#include "thread.h"
#include "client.h"

// Forward Declarations
namespace HAL
{
  class VisionInfo;
}

namespace Util
{
  class CS;
}

namespace Strategy
{
  class Kalman;
}

namespace Strategy
{
  class VisionThread : public Util::Thread
  {

  private:
    SSL_WrapperPacket recvPacket;
    static bool     instantiated;
    Kalman            *kFilter;
    Simulator::Client client;
    void run();

  public:
    VisionThread(Kalman* kFilter);
    ~VisionThread();
  }; // class VisionThread
} // namespace Strategy

#endif  // VISION_THREAD_H
