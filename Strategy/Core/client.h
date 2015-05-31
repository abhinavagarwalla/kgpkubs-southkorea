#ifndef CLIENT_H
#define CLIENT_H

#include <stdio.h>
#include <string>
#include "netraw.h"
#include "config.h"
#include "comdef.h"
#include "messages_robocup_ssl_detection.pb.h"
#include "messages_robocup_ssl_geometry.pb.h"
#include "messages_robocup_ssl_wrapper.pb.h"

namespace Simulator
{
  class Client
  {
  private:
    static const int    MaxDataGramSize = 65536;

    char                in_buffer[MaxDataGramSize];
    int                 port;
    std::string         address;
    std::string         inter_face;
    Net::UDP            multiCast;
    long int            frameNo;
    int                 tcapture;
    SSL_DetectionFrame  dframe;
    SSL_DetectionBall*  ball;
    SSL_DetectionRobot* bot[BlueTeam::SIZE + YellowTeam::SIZE];

  public:
    SSL_DetectionFrame* nframe;
    SSL_GeometryData*   gdata;

    Client(int port = Simulator::VISION_PORT,
           std::string address = Simulator::ADDRESS,
           std::string inter_face = Simulator::INTER_FACE);
           
    ~Client();
    
    bool openMulticast(bool block = false);
    
    void closeMulticast();
    
    bool receive(SSL_WrapperPacket& packet);
    
    void updateBot(TeamColor teamColor, int botID, float x, float y, float orientation);
    
    void updateBall(float x, float y);
  }; // class Client
} // namespace Strategy

#endif // CLIENT_H
