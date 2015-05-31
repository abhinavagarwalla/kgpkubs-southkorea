#include "fira_comm.h"
#include "serial.h"
#include "cs.hpp"
#include "logger.h"
#include "skillSet.h"
#include <sys/time.h>
#include <unistd.h>
using namespace std;
using namespace Util;
// Blue Bots Channel - D
namespace HAL
{
#ifdef COMBINED_PACKET
  CS FIRAComm::cs_internal[5];
  void FIRAComm::getSentData(int botid, int &vl, int &vr)
  {
    cs_internal[botid].enter();
    vl = command.data[botid*2];
    vr = command.data[botid*2+1];
    cs_internal[botid].leave();    
  }
#endif
  FIRAComm::FIRAComm()
  {
    debug_cs = new CS();
    if (!sPort.Open("/dev/ttyUSB0", 38400))
    { 
      Logger::abort("Could not open COMM port");
    }
    for(int i = 0; i < 5; i++)
    {
      lastVR[i] = 0;
      lastVL[i] = 0;
    }
    fp = fopen("trans.log", "w");
    openLoop[0] = false;
    openLoop[1] = false;
    openLoop[2] = false;
    openLoop[3] = false;
    openLoop[4] = false;
  }

  FIRAComm::~FIRAComm()
  {}
  
  void FIRAComm::whenBotSendsData(int ourV_l, int ourV_r)
  {
    FILE *f = fopen("/home/robo/botplot/compare_dataset/response.txt", "a");
    unsigned char botSendData = 0;
    bool ok, really = true;
    int botSendvL = sPort.ReadByteTimeout(4.0, ok);
    really &= ok;
    int botSendvR = sPort.ReadByteTimeout(4.0, ok);
    really &= ok;
    if(!really) {
      sPort.Clear();
    }
    fprintf(f, "%d %d %d %d %d\n", really, ourV_l, ourV_r, botSendvL, botSendvR);
    //printf(">>>>>>>>>>>Bot sent data: %d %d, really = %d\n", botSendvL , botSendvR, (int) really);
    fclose(f);
  }
#ifdef COMBINED_PACKET
  void FIRAComm::writeCombinedPacket()
  {
      for(int i=0; i<5; i++) {
        cs_internal[i].enter();
      }
      command.preamble = Strategy::HomeTeam::COLOR == Simulator::BLUE_TEAM ? (int8_t)126 : (int8_t)127;
	  command.timestamp = 0;
      sPort.Write(&command, sizeof(CombinedFIRAPacket));
      
#ifdef BOTLOG
      int reqBotId = 4;
      whenBotSendsData(command.data[reqBotId*2], command.data[reqBotId*2+1]); //need to think of how to implement this now
#endif
      for(int i=4; i>=0; i--) {
        cs_internal[i].leave();
      }
  }
#endif
  
  void FIRAComm::sendCommand(int botID,
                             float v_l,
                             float v_r)
  {
	/*Arpit: I deleted all the commented code for fake kalman on vL, vR. Plz refer to prev commits if needed later on. */
    int pwmWL = v_l;
    int pwmWR = v_r;
    printf("Bot Velocity(firacomm) %d: %d %d\n", botID, pwmWL, pwmWR);
#ifdef COMBINED_PACKET
    cs_internal[botID].enter();		
    command.data[botID*2] = (int8_t)pwmWL;
    command.data[botID*2+1] = (int8_t)pwmWR;
    cs_internal[botID].leave();
#else
    commCS.enter();
    command.preamble  = 0xAA;
    command.teamColor = (uint8_t)Strategy::HomeTeam::COLOR;
    command.botID     = botID;
    command.dirWL     = pwmWL < 0 ? 1 : 0;
    command.dirWR     = pwmWR < 0 ? 1 : 0;

    //printf("v_l: %f v_r: %f l:%d, r:%d\n", v_l, v_r, pwmWL, pwmWR);
    
    if (pwmWL < 0)
    {
      pwmWL = -pwmWL;
    }
    if (pwmWR < 0)
    {
      pwmWR = -pwmWR;
    }
    command.pwmWL     = pwmWL;
    command.pwmWR     = pwmWR;

    sPort.Write(&command, sizeof(FIRAPacket));
    commCS.leave();
#endif
  }
  void FIRAComm::addCircle(int x, int y, unsigned int radius, unsigned int color = 0xFFFFFFFF)
  {
    Debug_Circle circle;
    circle.set_x(x);
    circle.set_y(y);
    circle.set_radius(radius);
    circle.set_color(color);
    debug_cs->enter();
    debug_circles.push_back(circle);
    debug_cs->leave();
  }
  void FIRAComm::addLine(int x1, int y1, int x2, int y2, unsigned int color = 0xFFFFFFFF)
  {
    Debug_Line line;
    line.set_x1(x1);
    line.set_y1(y1);
    line.set_x2(x2);
    line.set_y2(y2);
    line.set_color(color);
    debug_cs->enter();
    debug_lines.push_back(line);
    debug_cs->leave();

  }
  int FIRAComm::xpos = 0;
  int FIRAComm::ypos = 0;
  float FIRAComm::angle = 0;
}
