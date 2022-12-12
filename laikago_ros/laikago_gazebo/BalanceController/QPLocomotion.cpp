#include <iostream>
#include "QPLocomotion.h"
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
 
  Mat3<double> Kp_stance =  0 * Kp;

  Mat3<double> Kd;
  kd << 15, 0, 0,
        0, 15, 0,
        0, 0, 15;
  Mat3<double> Kd_stance = 0 * Kd;

  // for debug
  // kp.setZero();
  // kd.setZero();

 
  // calc gait
  //gait->setIterations(iterationsPerCyc, iterationCounter);
  //iterationCounter++;

  Vec4<double> contactStates;
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
 
  updateQP(data);
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
    data._legController->commands[foot].feedforwardForce << 0, 0, 0;
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
    data._legController->commands[foot].feedforwardForce = f_ff[foot];
    //std::cout << foot << " : " << f_ff[foot] << std::endl;
    firstSwing[foot] = true;
    }
  
  }

  //data._stateEstimator->setContactPhase(contactStates);
  
}

void QPLocomotion::updateQP(ControlFSMData& data){
  auto seResult = data._stateEstimator->getResult();
  auto& stateCommand = data._desiredStateCommand;

  double minForce = 5;
  double maxForce = 500;
  double contactStateScheduled[4];
  
  double minForces[4] = {minForce, minForce, minForce, minForce};
  double maxForces[4] = {maxForce, maxForce, maxForce, maxForce};

  double COM_weights[3] = {50, 50, 500};
  double Base_weights[3] = {800, 500, 100};
  double pFeet[12], p_act[3], v_act[3], O_err[3], rpy[3],v_des[3], p_des[3],
         omegaDes[3];
  double se_xfb[13];
  double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];
  double b_control[6];

  for(int i = 0; i < 4; i++){
    se_xfb[i] = seResult.orientation(i);
    contactStateScheduled[i] = data._gaitScheduler->gaitData.contactStateScheduled(i);
    //std::cout << contactStateScheduled[i] << " ";
  }
    //std::cout << "\n";
  for(int i = 0; i < 3; i++){
    p_act[i] = seResult.position(i);
    v_act[i] = seResult.vBody(i);

    se_xfb[4 + i] = seResult.position(i);
    se_xfb[7 + i] = seResult.omegaBody(i);
    se_xfb[10 + i] = seResult.vBody(i);

    kpCOM[i] = 80;   //_data->controlParameters->kpCOM(i);
    kdCOM[i] = 20;     //_data->controlParameters->kdCOM(i);
    kpBase[i] = 150;     //_data->controlParameters->kpBase(i);
    kdBase[i] = 50; //  _data->controlParameters->kdBase(i);
  }

  p_des[0] = stateCommand->data.stateDes[0];
  p_des[1] = stateCommand->data.stateDes[1];
  p_des[2] = 0.4;
 
  rpy[0] = 0;
  rpy[1] = 0;
  rpy[2] = stateCommand->data.stateDes[5];

  omegaDes[0] = 0;
  omegaDes[1] = 0;
  omegaDes[2] = stateCommand->data.stateDes[11];

  //Vec3<double> pFeetVec;
  Vec3<double> pFeetVecCOM;

  for (int leg = 0; leg < 4; leg++) {
    pFeetVecCOM =  seResult.rBody.transpose() *
    (data._quadruped->getHipLocation(leg) + data._legController->data[leg].p);


    pFeet[leg * 3] = pFeetVecCOM[0];
    pFeet[leg * 3 + 1] = pFeetVecCOM[1];
    pFeet[leg * 3 + 2] = pFeetVecCOM[2];
  }

  QP.set_alpha_control(0.1);
  QP.set_friction(0.6);
  QP.set_mass(19);
  //QP.set_mass(12); // for a1 robot
  QP.set_wrench_weights(COM_weights, Base_weights);
  QP.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
  QP.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
  QP.SetContactData(contactStateScheduled, minForces, maxForces);
  QP.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                      O_err, seResult.rpy(2));

  double fOpt[12];
  QP.solveQP_nonThreaded(fOpt);

  for(int leg = 0; leg < 4; leg++){
    f_ff[leg] << fOpt[leg * 3], fOpt[leg * 3 + 1], fOpt[leg * 3 + 2];
 //data._legController->commands[leg].feedforwardForce = seResult.rBody.transpose() * f_ff[leg];
  //  std::cout << "leg " << leg << " : " << f_ff[2] << std::endl;
  }
  
}