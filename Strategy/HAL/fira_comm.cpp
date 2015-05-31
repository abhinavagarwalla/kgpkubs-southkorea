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
  



  void FIRAComm::sendForOpenLoop(int botID, float v_l, float v_r)
  {

    static struct timeval startTime, endTime;
    gettimeofday(&endTime, NULL);
    long long int microseconds = (endTime.tv_sec - startTime.tv_sec) * 1000000 + ((int)endTime.tv_usec - (int)startTime.tv_usec);
    long long int milliseconds = microseconds / 1000;
    //printf(">>>>>>>>>>>>>>Milliseconds = %lld\n", milliseconds);
    startTime = endTime;
    int diff = 2;
    //v_l /=10;
    //v_r /=10;
    /* These are certain speed limit constraints comply with the cintrifugal force
     * If difference between the speeds of the wheel is too high ie the bot is making sharp turns,
     * The bot starts to rotate on one single wheel. So In such cases the speed is decreased accordingly
     */
    v_l /= MAX_BOT_SPEED;
    v_r /= MAX_BOT_SPEED;
    float mv = max(abs(v_l), abs(v_r));
    const float MAXV = 60;
    const float MINV = 2;
    float VB = MAXV;
    float del = abs(v_l - v_r);
    if(del >= 0.001)
    {
      VB = 10 / del;
    }
    if(VB > MAXV) VB = MAXV;
    else if(VB < MINV) VB = MINV;
    int pwmWL = (int)(v_l * VB);

    int pwmWR = (int)(v_r * VB);
    const int MAXDEL = 5;

    if(pwmWL > lastVL[botID] + MAXDEL)
    {
      pwmWL = lastVL[botID] + MAXDEL;
    }
    else if(pwmWL < lastVL[botID] - MAXDEL)
    {
      pwmWL = lastVL[botID] - MAXDEL;
    }
    if(pwmWR > lastVR[botID] + MAXDEL)
    {
      pwmWR = lastVR[botID] + MAXDEL;
    }
    else if(pwmWR < lastVR[botID] - MAXDEL)
    {
      pwmWR = lastVR[botID] - MAXDEL;
    }

//    if(pwmWL > lpwmL + diff) pwmWL = lpwmL + diff;
//    else if(pwmWL < (lpwmL - diff)) pwmWL = lpwmL - diff;
//    if(pwmWR > lpwmR + diff) pwmWR = lpwmR + diff;
//    else if(pwmWR < (lpwmR - diff)) pwmWR = lpwmR - diff;

    lastVR[botID] = pwmWR;
    lastVL[botID] = pwmWL;
#ifdef COMBINED_PACKET
    cs_internal[botID].enter();
    command.data[botID*2] = (int8_t)pwmWL;
    command.data[botID*2+1] = (int8_t)pwmWR;
    cs_internal[botID].leave();
#else    
    commCS.enter();
//  botID = 0;
    command.preamble  = 0xAA;
    command.teamColor = (uint8_t)Strategy::HomeTeam::COLOR;
    command.botID     = botID;
    command.dirWL     = pwmWL < 0 ? 1 : 0;
    command.dirWR     = pwmWR < 0 ? 1 : 0;

    //printf(">>>>>>>>>>>>>>>>>>>>>v_l: %f v_r: %f l:%d, r:%d\n", v_l, v_r, pwmWL, pwmWR);
    //if(botID == 4)
    //  printf("l = %d, r = %d\n", pwmWL, pwmWR);
    //if(botID == 0)
    //  fprintf(fp, "%d %d %d %d %f\n",pwmWL, pwmWR, xpos, ypos, angle);
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
    //printf(">>>>>>>>>>>>>>>>>>>>>>>>>>>>SPEED : %d %d\n",pwmWL,pwmWR);
   
  }
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
