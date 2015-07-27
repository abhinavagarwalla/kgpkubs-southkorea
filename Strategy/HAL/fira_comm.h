#ifndef FIRACOMM_H
#define FIRACOMM_H

#include "comm.h"
#include "rbCommDef.h"
#include "serial.h"
#include "config.h"
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <netdb.h>
#include "geometry.hpp"
#include <vector>
#include "pathPlanners.h"
#include "sslDebug_Data.pb.h"

namespace Util
{
  class CS;
}

namespace HAL
{

  class FIRAComm : public Comm
  {
  private:
    Serial sPort;
    static Util::CS cs_internal[5]; /* Since each thread writes at a diff location, diff cs for each. However, all cs are entered while
                                     * data is written to serial port. Maybe think of a better soln?
                                     */
    CombinedFIRAPacket command;
    int lastVL[5];
    int lastVR[5];
    int lpwmL, lpwmR;
    bool openLoop[5];    
    FILE* fp;
  public:
//    std::vector<Strategy::obstacle> pts;
    static int xpos;
    static int ypos;
    static float angle;
    FIRAComm();
    ~FIRAComm();
	
    virtual void getSentData(int botid, int &vl, int &vr);
    void whenBotSendsData(int ourV_l, int ourV_r);
    void sendCommand(int   botID,
                     float v_l,
                     float v_r);
    // Adding dummy functions for compatibity with other codes.
    virtual void addCircle(int x, int y, unsigned int radius, unsigned int color);
    virtual void addLine(int x1, int y1, int x2, int y2, unsigned int color );    
    virtual void writeCombinedPacket();	
  }; //class FIRAComm

} // namespace HAL

#endif // FIRACOMM_H
