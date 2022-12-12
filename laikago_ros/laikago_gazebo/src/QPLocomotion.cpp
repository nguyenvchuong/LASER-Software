#include <iostream>
#include "../include/QPLocomotion.h"
#include "../include/Utilities/Timer.h"
#include "../include/Math/orientation_tools.h"

using namespace ori;

/* =========================== Controller ============================*/
QPLocomotion::QPLocomotion(double _dt, double gait_cycle_time) :
GaitCycleTime(gait_cycle_time),
dt(_dt)
{
  // std::cout << "init QP loco" << std::endl;
  rpy_comp[0] = 0;
  rpy_comp[1] = 0;
  rpy_comp[2] = 0;
  rpy_int[0] = 0;
  rpy_int[1] = 0;
  rpy_int[2] = 0;
  
  for(int i = 0; i < 4; i++){
    firstSwing[i] = true;
  }
}

void QPLocomotion::run(ControlFSMData& data)
{
  auto& seResult = data._stateEstimator->getResult();
  for(int i = 0; i < 4; i++){
  pFoot[i] = seResult.position +
          seResult.rBody.transpose() * (data._quadruped->getHipLocation(i) + data._legController->data[i].p);
  }
  Vec3<double> v_des_robot(0, 0, 0);
  Vec3<double> v_des_world(0, 0, 0);
  if(firstRun){
    // std::cout << "QP first run" << std::endl;
    data._gaitScheduler->gaitData._currentGait = GaitType::TROT;
    // data._gaitScheduler->gaitData.zero();
    // data._gaitScheduler->initialize();
    // data._gaitScheduler->gaitData.periodTimeNominal = GaitCycleTime;
    // data._gaitScheduler->createGait();
    // data._gaitScheduler->createGait();

    world_position_desired[0] = seResult.position[0];
    world_position_desired[1] = seResult.position[1];
    world_position_desired[2] = seResult.position[2];

    Vec3<double> v_des_robot(0,0,0);  // connect to desired state command later
    Vec3<double> v_des_world(0,0,0);  // connect to desired state command later

    Vec3<double> v_robot = seResult.vWorld;
    pBody_des[0] = world_position_desired[0];
    pBody_des[1] = world_position_desired[1];
    pBody_des[2] = world_position_desired[2];
    vBody_des[0] = v_des_world[0];
    vBody_des[1] = v_des_world[1];
    vBody_des[2] = 0;

    pBody_RPY_des[0] = 0;
    pBody_RPY_des[1] = 0;
    pBody_RPY_des[2] = seResult.rpy[2];

    vBody_Ori_des[0] = 0;
    vBody_Ori_des[1] = 0;
    vBody_Ori_des[2] = 0; // set this for now

    for(int i = 0; i < 4; i++){
      footSwingTrajectories[i].setHeight(0.1);
      footSwingTrajectories[i].setInitialPosition(pFoot[i]);
      footSwingTrajectories[i].setFinalPosition(pFoot[i]);
    }
    firstRun = false;
  }

  // double side_sign[4] = {-1, 1, -1, 1};
  // double interleave_y[4] = {-0.08, 0.08, 0.01, -0.01};
  // double interleave_gain = -0.2;
  // double v_abs = std::fabs(seResult.vBody[0]);
  // Vec3<double> des_vel(0,0,0);
  data._gaitScheduler->step();

  Mat3<double> Kp;
  kp << 500, 0, 0,
        0, 500, 0,
        0, 0, 150;
 
  Mat3<double> Kp_stance =  Kp;

  Mat3<double> Kd;
  kd << 15, 0, 0,
        0, 15, 0,
        0, 0, 15;
  Mat3<double> Kd_stance = Kd;

  // for debug
  // kp.setZero();
  // kd.setZero();

 
  // calc gait
  //gait->setIterations(iterationsPerCyc, iterationCounter);
  //iterationCounter++;

  Vec4<int> contactStates;
  Vec4<double> swingStates;
 

  for(int i = 0; i < 4; i++){
    contactStates(i) = data._gaitScheduler->gaitData.contactStateScheduled(i);
    
    swingStates(i) = data._gaitScheduler->gaitData.phaseSwing(i);
    swingTimes[i] = data._gaitScheduler->gaitData.timeSwing(i);
  }

   double side_sign[4] = {-1, 1, -1, 1};
  double interleave_y[4] = {-0.08, 0.08, 0.01, -0.01};
  double interleave_gain = -0.3;
  double v_abs = std::fabs(seResult.vBody[0]);
  for(int i = 0; i < 4; i++)
  {

    if(firstSwing[i]) {
      swingTimeRemaining[i] = swingTimes[i];
    } else {
      swingTimeRemaining[i] -= dt;
    }

    
    if(firstSwing[i]) {
      
      footSwingTrajectories[i].setHeight(.1);
      Vec3<double> offset(0, side_sign[i] * .083, 0);
      // simple heuristic function
      
      
      Vec3<double> pRobotFrame = (data._quadruped->getHipLocation(i) + offset);
      Vec3<double> pYawCorrected = coordinateRotation(CoordinateAxis::Z, 0. /*-stateCommand->data.stateDes[11] * gait->_stance * dtMPC / 2*/) * pRobotFrame;
 
      Vec3<double> des_vel;
      des_vel[0] = 0; //stateCommand->data.stateDes(6);
      des_vel[1] = 0; //stateCommand->data.stateDes(7);
      des_vel[2] = 0; //stateCommand->data.stateDes(8);
      Vec3<double> Pf = seResult.position +
                       seResult.rBody.transpose() * (pYawCorrected
                       + des_vel * swingTimeRemaining[i]);

      //+ seResult.vWorld * swingTimeRemaining[i];

      double p_rel_max = 0.4;
      double pfx_rel = seResult.vWorld[0] * .5 * data._gaitScheduler->gaitData.timeStance[i]  +
                      .03*(seResult.vWorld[0]-v_des_world[0]) +
         (0.5*seResult.position[2]/9.81) * (seResult.vWorld[1]*0);
      double pfy_rel = seResult.vWorld[1] * .5 *data._gaitScheduler->gaitData.timeStance[i]  +
                      .03*(seResult.vWorld[1]-v_des_world[1]) +
                      (0.5*seResult.position[2]/9.81) * (-seResult.vWorld[0]*0);
      pfx_rel = fminf(fmaxf(pfx_rel, -p_rel_max), p_rel_max);
      pfy_rel = fminf(fmaxf(pfy_rel, -p_rel_max), p_rel_max);
      Pf[0] +=  pfx_rel;
      Pf[1] +=  pfy_rel + interleave_y[i] * v_abs * interleave_gain;
      //Pf[2] = -0.01;
      Pf[2] = 0.0;
      
      footSwingTrajectories[i].setFinalPosition(Pf);
    }
    

  }

  // std::cout << "contact \n" << contactStates << std::endl;
  // std::cout << "swing \n" << swingStates << std::endl;
  // Vec4<double> swingStates = gait->getSwingState();
 

  Vec4<double> se_contactState(0,0,0,0);
  
  for(int foot = 0; foot < 4; foot++)
  {
    double swingState = swingStates(foot);
    int contactState = contactStates(foot);
    footSwingTrajectories[foot].computeSwingTrajectoryBezier(swingState, swingTimes[foot]);
    // std::cout << "swingState " << foot << ": " << swingState << std::endl;
    se_contactState(foot) = contactState;
    Vec3<double> pDesFootWorld = footSwingTrajectories[foot].getPosition();
    Vec3<double> vDesFootWorld = footSwingTrajectories[foot].getVelocity();
    Vec3<double> pDesLeg = seResult.rBody * (pDesFootWorld - seResult.position) 
        - data._quadruped->getHipLocation(foot);
    Vec3<double> vDesLeg = seResult.rBody * (vDesFootWorld - seResult.vWorld);
    // std::cout << "pDesleg" << foot << ": \n" << pDesLeg << std::endl;
    // std::cout << "vDesleg" << foot << ": \n" << vDesLeg << std::endl;
    data._legController->commands[foot].pDes = pDesLeg;
    data._legController->commands[foot].vDes = vDesLeg;
    data._legController->commands[foot].kpCartesian = kp;
    data._legController->commands[foot].kdCartesian = kd;
    firstSwing[foot] = false;
    if(contactState == 1){
    data._legController->commands[foot].kpCartesian = kp_stance;
    data._legController->commands[foot].kdCartesian = kd_stance;
    data._legController->commands[foot].pDes = pDesLeg;
    data._legController->commands[foot].vDes = vDesLeg;
    firstSwing[foot] = true;
    }
  
  }

  data._stateEstimator->setContactPhase(se_contactState);
}
