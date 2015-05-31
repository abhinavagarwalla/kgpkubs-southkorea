/* This file gets the position of the robots from SSL Vision
 * and stores then in a HAL::VisionInfo class.
 * It is also responsible for doing the post processing in estimating the
 * robot position and velocities after negotiating the latency from the
 * point of frame capture to the point of sending commands to the robots.
 */

#include "comdef.h"
#include "rbCommDef.h"
#include "visionThread.h"
#include "cs.hpp"
#include "logger.h"
#include "kalman.h"

using namespace std;
using namespace Util;
using namespace HAL;
using namespace Simulator;

namespace Strategy
{
  VisionThread::VisionThread(Kalman *_kFilter):kFilter(_kFilter)
  {
  }

  VisionThread::~VisionThread()
  { }

  void VisionThread::run()
  {
    while (true)
    {
      if (client.receive(recvPacket))
      {
        if (recvPacket.has_detection())
        {
          SSL_DetectionFrame detection = recvPacket.detection();
          kFilter->addInfo(detection);

#ifdef SHOW_SSLVISION_LOG
            /*Logger::toStdOut("Vision Frame = %d ID = %d Angle = %f v_x = %f v_y = %f v_t = %f delay = %f\n",
                             viSh->frameNum,
                             0,
                             viSh->blueAngle[0],
                             viSh->blueVel[0].x,
                             viSh->blueVel[0].y,
                             viSh->blueAngVel[0]
               );*/
#endif // SHOW_SSLVISION_LOG
        }

        /* Once a frame is received, the next frame will be received atleast after 1000/60ms ~ 16ms
         * due to the camera frame rate is capped at 60fps
         */
        sleep(10);
      }
      // Sleep for 1ms to prevent CPU hogging
      sleep(1);
    }
  }
} // namespace Strategy
