#ifndef FSM_State_H
#define FSM_State_H

#include <stdio.h>
#include <fstream>
#include "body.h"
#include "TransitionData.h"
#include "ControlFSMData.h"
#include "GaitScheduler.h"
//#include "QPLocomotion.h"
#include "../ConvexMPC/ConvexMPCLocomotion.h"
#include "../BalanceController/BalanceController.hpp"
#include "../LeastSquareSolution/LeastSquareSolution.hpp"

class FSM_State{
  public:
    FSM_State(ControlFSMData* _controlFSMData);
    
    // void runControl();  // needed when there is other high-level controllers
    void QPstand(); // stand with QP controller
    void runBalanceController(); // QP controller

    void PDstand(); // stand up with PD controller
    //double* p_des_in, double* rpy_des_in
          //            double* v_des_in, double* omega_des_in
    void runLeastSquare(); // least sqaure solution

    // Holds all off the relevant control data
    ControlFSMData* _data;

    double transitionDuration;  // transition duration time
    double tStartTransition;    // time transition starts
    TransitionData transitionData;

    
    // Pre controls safety checks
    bool checkSafeOrientation = false;  // check roll and pitch

    // Post control safety checks
    bool checkPDesFoot = false;          // do not command footsetps too far
    bool checkForceFeedForward = false;  // do not command huge forces
    bool checkLegSingularity = false;    // do not let leg

    // Leg controller command placeholders for the whole robot (3x4 matrices)
    Mat34<double> jointFeedForwardTorques;  // feed forward joint torques
    Mat34<double> jointPositions;           // joint angle positions
    Mat34<double> jointVelocities;          // joint angular velocities
    Mat34<double> footFeedForwardForces;    // feedforward forces at the feet
    Mat34<double> footPositions;            // cartesian foot positions
    Mat34<double> footVelocities;           // cartesian foot velocities

    // Footstep locations for next step
    Mat34<double> footstepLocations; 

    // Higher level Robot body controllers
    BalanceController balanceController;
    LeastSquareSolution leastSquareController;
    ConvexMPCLocomotion Cmpc;
    
    //QPLocomotion locomotionController;
    // ModelPredictiveController cMPC
    // RegularizedPredictiveController RPC

    bool runQP = true;

 private:
    // Create the cartesian P gain matrix
    Mat3<double> kpMat;

    // Create the cartesian D gain matrix
    Mat3<double> kdMat;


};


#endif