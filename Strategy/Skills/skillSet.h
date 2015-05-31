// It is through the skill layer that the Strategy layer talks to the Simulator/Actual robots
#pragma once
#ifndef SKILLS_H
#define SKILLS_H
#include "evaluator.h"
#include "dlib/svm.h"

#if GR_SIM_COMM
# include "grSimComm.h"
#elif FIRASSL_COMM||FIRA_COMM
# include "fira_comm.h"
#else
# error Macro for Comm class not defined
#endif

#include <vector>
#include "pathPlanners.h"
#include "sslDebug_Data.pb.h"
// Forward Declarations
namespace Strategy
{
  class ERRT;
  class MergeSCurve;
  class LocalAvoidance;
  class BeliefState;
}
namespace HAL {
  class Comm;
}
template <class T> class Vector2D;
namespace Strategy
{
  /* Procedure for adding a skill successfully to the SkillSet is:
   * 1. Enumarate it in enum SkillSet::SkillID just before MAX_SKILLS
   * 2. Identify its input parameters and define a struct for it in union SParam
   * 3. Declare the function prototype for the skill. Its parameter must only be SParam& param
   * 4. Map the Skill ID to the corresponding function in the constructor of SkillSet
   * 5. Define the function that implements the skill declared in step 3.
   * NOTE: Variable names of enums and corresponding skill function names must be the same,
   *       except that the enum name starts with a capital letter. Name of the corresponding
   *       struct should be the same as that of enum with a trailing 'P' character
   */
  // Union of parameters for each skill enumerated in SkillID
    union SParam
    {
      // Parameters for the skill Spin
      struct type1
      {
        float radPerSec;
      } SpinP;

      // Parameters for the skill Stop
      struct type2
      {} StopP, GoalKeepingP,;

      // Parameters for the skill Kick
      struct type3
      {
        float power;
      } KickP;

      // Parameters for the skill Velocity
      struct type4
      {
        float vl;
        float vr;
      } VelocityP;

      //Parameters for skill goToPoint and dribble to point
      struct type5
      {
        float x;
        float y;
        float finalslope;
        float radius;
      } DribbleToPointP, TurnToPointP, DefendPointP;
      struct type6
      {
        float finalslope;
      } TurnToAngleP;
      struct type7
      {
        bool align;
      } GoToBallP;
      struct type8
      {
        float x;
        float y;
        float finalslope;
        bool align;
        float finalVelocity;
		//Parameters for learning?? not needed
        float bot_point_dist_penalty;
        float angle_diff_penalty;
        bool increaseSpeed;
        bool isBallObstacle ;
      } GoToPointP;
    };
  class SkillSet
  {
  public:
    enum SkillID
    {
      Spin,
      SpinToGoal,
      Stop,
      Velocity,
      GoToBall,
      GoToPoint,
      GoalKeeping,
      DribbleToPoint,
      TurnToPoint,
      TurnToAngle,
      DefendPoint,
//      DribbleToGoal,
      GoToPointStraight,
      GoToBallStraight,
      GoToPointGoalie,   //BOT_POINT_THESH reduced for Goalie
			ChargeBall,
      MAX_SKILLS
    };
    static HAL::Comm*    comm;
  protected:
    typedef void (SkillSet::*skillFncPtr)(const SParam&);
    const BeliefState* state;
    const int          botID;
    skillFncPtr        skillList[MAX_SKILLS];
#ifdef LOCAL_AVOID
    LocalAvoidance*    pathPlanner;
    MergeSCurve*       pathPlannerSec;
#else
    MergeSCurve*       pathPlanner;
#endif
    ERRT*              errt;
    
    /* TODO: Extra Shit, relevance needs to be verified
     */
    bool pointyInField(Vector2D<int> fianl);
    void _goToPoint(int botid, Vector2D<int> dpoint, float finalvel, float finalslope, float clearance,bool increaseSpeed=0);
    void _goToPointLessThresh(int botid, Vector2D<int> dpoint, float finalvel, float finalslope, float clearance,bool increaseSpeed=0);
    void _dribbleToPoint(int botID, Vector2D<int> dpoint, float finalvel, float finalslope, float clearance);
    void _turnToAngle(float angle, float *vl, float *vr);
    void _goToPointStraight(int botid, Vector2D<int> dpoint, float finalvel, float finalslope, float clearance);
    int prevVel;
    /***************************By Prasann***********/
    /* Arpit: params removed from here, made static in function itself. I wanted to make function static so that
     * it can be called from sendCommand or tester itself. */
    /***************************By Prasann***********/
  public:
    //------- List of robot skills -------//
    void spin(const SParam& param);
    void stop(const SParam& param);
    //void kick(const SParam& param);
    //void dribble(const SParam& param);
    void velocity(const SParam& param);
    void goToBall(const SParam& param);
    void goToPoint(const SParam& param);
    void goToPointGoalie(const SParam& param);
    void dribbleToPoint(const SParam& param);
    void receiveBall(const SParam& param);
    void goalKeeping(const SParam& param);
    void spinToGoal(const SParam& param);
    //void kickToGoal(const SParam& param);
    void turnToPoint(const SParam& param);
    void turnToAngle(const SParam& param);
    void defendPoint(const SParam& param);
    void dribbleToGoal(const SParam& param);
    void goToPointStraight(const SParam& param);
    void goToBallStraight(const SParam& param);
		void chargeBall(const SParam& param);
    
    // Parameter for skills to be trained
    static bool loadParamsFromFile;
    static bool  skillParamsLoaded;
    static float GoToPoint_profileFactorMult;
    static float GoToPoint_omegaFactor;
    static float Bot_Point_dis_penalty_l1;
    static float Bot_Point_dis_penalty_l2;
    static float angle_penalty_l1;
    static float angle_penalty_l2;
    static dlib::decision_function<dlib::radial_basis_kernel<dlib::matrix<double, 3, 1> > > gotopoint_profilefac;
    static dlib::decision_function<dlib::radial_basis_kernel<dlib::matrix<double, 3, 1> > > gotopoint_omegafac;
    static int dribble_ball_threshold;
    static int bot_ball_threshold;
    static std::string skillsCollection;
    static std::string goToPointCollection;
    /* Error Function by
     * Soumyadeep & Ankit
     */
    static void errorLog(int botid,float vl,float vr, const BeliefState *state);
    
    
    SkillSet(const BeliefState* state, int botID);
    ~SkillSet();
    // Executing the skill function corresponding to the given ID and param function parameters
    inline void executeSkill(SkillID ID, const SParam& param)
    {
      (*this.*skillList[ID])(param);
    } // executeSkill
    
    //QtDebugger Compatibility Functions
	static void addCircle(int x, int y, unsigned int radius, unsigned int color)
	{
		comm->addCircle(x, y, radius, color);
	}
	static void addLine(int x1, int y1, int x2, int y2, unsigned int color )
	{
		comm->addLine(x1, x2, y1, y1, color);
	}
    static std::list<Debug_Circle> getCircles()
    {
      return comm->getCircles();
    }
    static std::list<Debug_Line> getLines()
    {
      return comm->getLines();
    }
    static void clearDebugData()
    {
#ifdef FIRASSL_COMM
      comm->clearDebugData();
#endif
    }
  }; // class SkillSet
} // namespace Strategy
#endif // SKILLS_H
