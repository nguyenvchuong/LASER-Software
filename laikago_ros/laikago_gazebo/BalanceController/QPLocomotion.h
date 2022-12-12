#ifndef QP_LOCOMOTION_H
#define QP_LOCOMOTION_H

#include "../include/FootSwingTrajectory.h"
#include "BalanceController.hpp"
#include "../include/cppTypes.h"
#include "../include/ControlFSMData.h"

using Eigen::Array4d;
using Eigen::Array4i;

/*enum class GaitType{
  STAND,
  TROT
};*/

/*struct GaitData {
  GaitData() { zero();}

  // zero all data
  void zero();

  GaitType _currentGait;
  GaitType _nextGait;
  std::string gaitName;

  // gait description
  double periodTimeNominal;       // overall period time to scale
  double initialPhase;            // initial phase to offset
  double switchingPhaseNominal;   // nominal phase to switch contacts

  // enable flag
  Eigen::Vector4i gaitEabled;     // enable gait controlled legs

  // Time-based description
  Vec4<double> periodTime;     // overall foot scaled gait period time
  Vec4<double> stanceTime;
  Vec4<double> swingTime;
  Vec4<double> stanceTimeRemain;
  Vec4<double> swingTimeRemain;

  // Phase description
  Vec4<double> switchingPhase;
  Vec4<double> phaseVar;      // overall gait phase for each foot
  Vec4<double> phaseOffset;   // nomial gait phase offsets
  Vec4<double> phaseScale;    // phase scale relative to variable
  Vec4<double> stancePhase;
  Vec4<double> swingPhase;

  // contacts
  Eigen::Vector4i contactScheduled;
  Eigen::Vector4i contactPrev;
  Eigen::Vector4i touchdownScheduled;
  Eigen::Vector4i liftoffSchedulred;
  
  Mat34<double> posFootLiftoffWorld;
  Mat34<double> posFootTouchdownWorld;
};

class Gait{
  public:
    Gait(double _dt);
    ~Gait();

    void initialize();
    void step();
    // deifine gait from predefined library
    void createGait();

    GaitData gaitData;
  private:
    double dt; // cotrol loop timestep
    double dphase; // phase change at each step
};*/

class QPLocomotion {
  public:
    QPLocomotion(double _dt, double _iterations_per_cycle);
    void initialize();

    void run(ControlFSMData& data);
 
    Vec3<double> pBody_des;
    Vec3<double> vBody_des;
    Vec3<double> aBody_des;

    Vec3<double> pBody_RPY_des;
    Vec3<double> vBody_Ori_des;

    Vec3<double> pFoot_des[4];
    Vec3<double> vFoot_des[4];
    Vec3<double> aFoot_des[4];

    Vec3<double> Fr_des[4];

    Vec4<double> contact_state;

  private:
    BalanceController QP;
    void updateQP(ControlFSMData& data);
    double GaitCycleTime;
    //int horizonLength;
    double dt;
    int iterationCounter = 0;
    Vec3<double> f_ff[4];
    Vec4<double> swingTimes;

    FootSwingTrajectory<double> footSwingTrajectories[4];
    bool firstSwing[4];
    //Gait trotting, bounding, walking;
    double swingTimeRemaining[4];
    Mat3<double> kp, kd, kp_stance, kd_stance;
    
    Vec3<double> world_position_desired;
    Vec3<double> rpy_int;
    Vec3<double> rpy_comp;
    Vec3<double> pFoot[4];


    bool firstRun = true;
  
};

#endif