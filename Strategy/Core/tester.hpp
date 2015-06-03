#include "mainheaders.hpp"
#include "thread.h"
#include "strategygui_thread.hpp"
#include "command-packet.pb.h"
#include "tExec.h"

#include <fstream>

using namespace std;
using namespace Strategy;


class Tester : public Util::Thread
{
public:
  bool &running;
  BeliefState &state;
  Tester(bool &running_, BeliefState &state_):
  running(running_),state(state_) {}
  void run()
  {
    Util::CS loggerCS;
    Util::Logger logger("timelog.log", Util::Logger::Write, loggerCS);
    Kalman      kFilter;
    VisionThread vThread(&kFilter);
    vThread.start();
	
    #ifdef STR_GUI
    Util::CS strCS;
    StrategyPacket strPktSh;
    StrategyGUIThread strGUIThread = StrategyGUIThread::getInstance(&strPktSh, &strCS);
    strGUIThread.start();
	TExec tExec(&state);
	int prev_tactic_id = -1;
	int prev_bot_id = -1;
	int prev_play_id = -1;
	PExec       pExec(&state);
    #endif
	
	Tactic::Param paramList[Tactic::MAX_TACTICS];
	
    //tGoalKeeping
    TGoalKeepingOurSide tGoalOur0(&state, 0);
    TGoalKeepingOurSide tGoalOur1(&state, 1);
	TGoalKeepingOurSide tGoalOur2(&state, 2);
	TGoalKeepingOurSide tGoalOur3(&state, 3);
	TGoalKeepingOurSide tGoalOur4(&state, 4);
    Tactic::Param paramGoal;
    
    //tCharge
    TCharge tCharge2(&state, 2);
    TCharge tCharge1(&state, 1);
    Tactic::Param pCharge;
    
    //tReceiveBall
    TReceiveBall tReceive3(&state, 3);
    TReceiveBall tReceive0(&state, 0);
		TReceiveBall tReceive1(&state, 1);
		TReceiveBall tReceive2(&state, 2);
    Tactic::Param pReceive;
    
    \
    //tCover Bot
    TCoverGoal tcover0(&state, 0);
    TCoverGoal tcover2(&state,2);
	TCoverGoal tcover3(&state,3);
    Tactic::Param paramcover;
    paramcover.CoverGoalP.distFromGoal = -2*DBOX_WIDTH;
    paramList[Tactic::CoverGoal].CoverGoalP.distFromGoal = -2*DBOX_WIDTH;
	
    //tStop Bot
    TStop tS0(&state, 0);
    TStop tS1(&state, 1);
    TStop tS2(&state, 2);
    Tactic::Param paramStop;
    
    //tposition
    TPosition tPos1(&state, 1);
    TPosition tPos2(&state,2);
    Tactic::Param pPos;
    paramList[Tactic::Position].PositionP.align = false;
    paramList[Tactic::Position].PositionP.x= 0;
    paramList[Tactic::Position].PositionP.y= 0;
    paramList[Tactic::Position].PositionP.finalSlope=PI/2;
    
    //tDefendLine
    TDefendLine tDefendLine1(&state,1);
    TDefendLine tDefendLine2(&state,2);

    Tactic::Param pDefendL1;
    paramList[Tactic::DefendLine].DefendLineP.x1 = 0;//BOT_RADIUS/2;
    paramList[Tactic::DefendLine].DefendLineP.y1 = -HALF_FIELD_MAXY/2;//-BOT_RADIUS/2;;
    paramList[Tactic::DefendLine].DefendLineP.x2 = 0;
    paramList[Tactic::DefendLine].DefendLineP.y2 = HALF_FIELD_MAXY/2;//-HALF_FIELD_MAXY;
	
	
    Tactic::Param pDefendL2;
    pDefendL2.DefendLineP.x1 = BOT_RADIUS/2;
    pDefendL2.DefendLineP.x2 = HALF_FIELD_MAXX/2;
    pDefendL2.DefendLineP.y1 = BOT_RADIUS/2;
    pDefendL2.DefendLineP.y2 = HALF_FIELD_MAXY;
    
    //tVelocity
    TVelocity tVelocity0(&state,0);
		TVelocity tVelocity1(&state,1);
		TVelocity tVelocity3(&state,3);
		TVelocity tVelocity4(&state,4);
		TVelocity tVelocity2(&state,2);
    
	Tactic::Param pVelocity;
	pVelocity.VelocityP.vl = 60;
	pVelocity.VelocityP.vr = 60;
	
	Tactic::Param pVelocity_1;
	pVelocity_1.VelocityP.vl = 20;
	pVelocity_1.VelocityP.vr = 20;
	
	Tactic::Param pVelocity_2;
	pVelocity_2.VelocityP.vl = 60;
	pVelocity_2.VelocityP.vr = 60;
	Tactic::Param pVelocity_3;
	pVelocity_3.VelocityP.vl = 60;
	pVelocity_3.VelocityP.vr = 60;
	Tactic::Param pVelocity_4;
	pVelocity_4.VelocityP.vl = 80;
	pVelocity_4.VelocityP.vr = 80;
	
    paramList[Tactic::Velocity].VelocityP.vl = 60;
    paramList[Tactic::Velocity].VelocityP.vr = 60;
  
    //tDefend Point
    TDefendPoint tDefendPoint1(&state,1);
    Tactic::Param pDefendPoint;
    paramList[Tactic::DefendPoint].DefendPointP.x = -HALF_FIELD_MAXX/2;
    paramList[Tactic::DefendPoint].DefendPointP.y = 0;
    paramList[Tactic::DefendPoint].DefendPointP.radius = HALF_FIELD_MAXX/4;
    
    //tAttack
    TAttack tAttack1(&state, 1);
    TAttack tAttack0(&state, 0);
    TAttack tAttack2(&state, 2);
    TAttack tAttack3(&state, 3);
    TAttack tAttack4(&state, 4);
    Tactic::Param pAttack;
    paramList[Tactic::Attack].AttackP.rotateOnError = true;
    
    // TestgotoPoint
    Strategy::Testgotopoint ttest1(&state,1);
	
	//SplineGoToPoint
	Strategy::SParam params1;
	params1.SplineGoToPointP.finalVelocity = 0;
	params1.SplineGoToPointP.x = -HALF_FIELD_MAXX;
	params1.SplineGoToPointP.y = 0;
	params1.SplineGoToPointP.finalslope = PI/2 ;
	params1.SplineGoToPointP.initTraj = 1;
	SkillSet sppoint(&state, 1); 
	
	Strategy::SParam params2;
	SkillSet point(&state, 0);
	params2.GoToPointP.x = 0;
	params2.GoToPointP.y = 0;
	params2.GoToPointP.finalslope = PI/2;
	
	
    Tactic::Param ptestpoint;
    TestbotRace ttest2(&state,2);
    Tactic::Param ptestrace;
    ptestrace.TestbotRaceP.vl = 100;
    ptestrace.TestbotRaceP.vr = 100;
   // FILE *f = fopen("/home/robo/ballPos.txt", "w");  
#ifdef BOTLOG
    FILE *f = fopen("/home/robo/botplot/compare_dataset/botlog.txt", "w");    
    fclose(f);
    f = fopen("/home/robo/botplot/compare_dataset/response.txt", "w");
    fclose(f);
#endif
    bool isRunning = true; 
    //    Util::Timer timer;
	int loopcount = 0;
    while(running)
    {
    //      timer.start();
      state.update();
      kFilter.update(state);
      
      if(1)
      {
		#ifdef STR_GUI
		{
			int test_tactic=0;
			if(test_tactic){
				strCS.enter();
				//if(strPktSh.which()==1)printf("1\n\n");
				if(strPktSh.tactic().tid() != prev_tactic_id){
					prev_tactic_id = strPktSh.tactic().tid();
					printf("\n\n\n\n\n\n\n");
					printf("****************  HELLO  ******************");
					printf("\n\n\n\n\n\n\n");
				}
				else{
					//printf("****************  BYE BYE  ******************");
				}
		  
				if(strPktSh.tactic().botid() != prev_bot_id){
					prev_bot_id = strPktSh.tactic().botid();
				}
			  
				strCS.leave();
				tExec.execute(&state,(Tactic::ID)(prev_tactic_id-1),paramList[prev_tactic_id -1],prev_bot_id);
				cout << "\n\n From tester: "<< prev_tactic_id-1 << "\t\t" << prev_bot_id;
			}
		
			else{
				// Critical Section protected by refBoxCS
				strCS.enter();
				// Updating from Referee Box and the Game State
				//  printf("refBoxCmdSh.cmdCounter = %d\n",refBoxCmdSh.cmdCounter);
				if (strPktSh.play() != prev_play_id)
				{
					printf("GOT COMMAND FROM STRATEGY GUI!!!!\n");
					prev_play_id = strPktSh.play();
					Util::Logger::toStdOut("Command From Refee.. Reselecting play..\n");
					pExec.selectfromGUI(prev_play_id);
				}
				pExec.executePlay();
				strCS.leave();
			}
		}
		#endif
	  
        //printf("Ball Pos: %d %d\n",state.ballPos.x,state.ballPos.y);
		
			//	printf(" \n\n %d %d \n\n",state.ourGoalCount,state.oppGoalCount);
		
        printf("our side %d\n",state.pr_ballOurSide);
        //printf("opp side %d\n",state.pr_ballOppSide);
        //printf("mid %d\n",state.pr_ballMidField);
        //printf("dbox %d\n",state.pr_ball_in_opp_dbox);    
        //ttest1.execute(ptestpoint);
        //tVelocity2.execute(pVelocity);
	  // tS0.execute(paramStop);
	  //tAttack2.execute(pAttack);
    //tAttack4.execute(pAttack); 

	 //tVelocity1.execute(pVelocity);
	//tVelocity0.execute(pVelocity);
	/*	tVelocity2.execute(pVelocity_2);
   	tVelocity3.execute(pVelocity_3);  
	  tVelocity4.execute(pVelocity_4);
		*/		//tVelocity1.execute(pVelocity);
				//tVelocity3.execute(pVelocity);
				//tAttackDuo12.execute(pAttack);

      //  tGoalOur2.execute(paramGoal);
        //tGoalOur1.execute(paramGoal);
        //tDefendLine1.execute(pDefendL1);
				//tGoalOur2.execute(paramGoal);
	//	tGoalOur3.execute(paramGoal);
		sppoint.executeSkill(SkillSet::SplineGoToPoint, params1);
		params1.SplineGoToPointP.initTraj = 0;
        //tPosition
				//tcover3.execute(paramcover);
       // tAttack3.execute(paramcover);
   //     tcover0.execute(paramcover);
        //goTopointStraight.executeSkill(SkillSet::GoToBallStraight,param1);
        //goToBallStraight.executeSkill(SkillSet::GoToBallStraight,param9)
     //   tAttack2.execute(pAttack);
			//	tReceive3.execute(pReceive);
				//tReceive1.execute(pReceive);
	//	tAttack3.execute(pAttack);
	 //tAttack1.execute(pAttack);
        //tAttack4.execute(pAttack);
        //tcover0.execute(paramcover);
        //tcover3.execute(paramcover);
//        goTopointStraight.executeSkill(SkillSet::GoToBallStraight,param1);
        //goToBallStraight.executeSkill(SkillSet::GoToBallStraight,param9);
        // goTopointStraight.executeSkill(SkillSet::GoToPointStraight,param1);
     //tCharge1.execute(pCharge);
		//	tAttack4.execute(pAttack);
		//	tAttack2.execute(pAttack);
		//	tAttack1.execute(pAttack);
			//tAttack3.execute(pAttack);
			//tAttack4.execute(pAttack);
		//tBackup0.execute(pBackup);
        //tDefendArc0.execute(pDefendArc);
//        tDefendLine1.execute(pDefendL1);
	//	tDefendLine2.execute(pDefendL2);
       //tAttack2.execute(pAttack);

        SkillSet::comm->writeCombinedPacket();
       
      }
     else
      {
        printf("OFF!\n");
        tS0.execute(paramStop);
        tS1.execute(paramStop);
        tS2.execute(paramStop);
      }
    //      printf("tIMER US = %d\n", timer.stopus() );
      //moprintf("Ball Pos: %d %d %f\n",state.ballPos.x,state.ballPos.y,state.homeAngle[2]);
         usleep(16000);  // Adding sleep to this thread of execution to prevent CPU hogging
      
    }
    vThread.stop();
    Util::Logger::toStdOut("Exiting process");
    return;

  }
};