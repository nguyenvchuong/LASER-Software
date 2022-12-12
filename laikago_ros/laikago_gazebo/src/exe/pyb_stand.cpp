/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/
/************************************************************************

Runs PDStand() and publishes lowState (full robot state)

This can then be received in pybullet, see:
    usc_learning/ros_interface/example_vis_ros_state.py

************************************************************************/

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "laikago_msgs/HighCmd.h"
#include "laikago_msgs/HighState.h"
#include "laikago_msgs/LowCmd.h"
#include "laikago_msgs/LowState.h"
#include "laikago_msgs/MotorCmd.h"
#include "laikago_msgs/MotorState.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <vector>
#include <string.h>
#include <math.h>
#include <nav_msgs/Odometry.h>
#include "../../include/body.h"
#include "../../include/OrientationEstimator.h"
#include "../../include/PositionVelocityEstimator.h"
#include "../../include/ContactEstimator.h"
#include "../../include/LegController.h"
#include "../../include/FSM.h"
#include "../../include/FootSwingTrajectory.h"
//#include "../../include/Dynamics/AlienGo.h"
#include "../../BalanceController/BalanceController.hpp"


using namespace std;
using namespace laikago_model;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "laikago_gazebo_servo");
    ros::Publisher lowCmd_pub, highCmd_pub; //for rviz visualization
    ros::NodeHandle n;
    ros::Publisher lowState_pub; //for rviz visualization
   // multiThread multi;
  /*!  ros::AsyncSpinner spin(1);
    spin.start();
    usleep(300000);
    
    ros::NodeHandle nm;
   // usleep(300000); // must wait 300ms, to get first state
    lowState_pub = nm.advertise<laikago_msgs::LowState>("/aliengo_gazebo/lowState/state", 1);
    servo_pub[0] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = nm.advertise<laikago_msgs::MotorCmd>("/aliengo_gazebo/RL_calf_controller/command", 1);

    motion_init();
    */
   
    ros::AsyncSpinner spinner(1); // one threads
    ros::Rate loop_rate(1000);
    spinner.start();

    CurrentState listen_publish_obj;

    usleep(300000); // must wait 300ms, to get first state
   
    

    // the following nodes have been initialized by "gazebo.launch"
    lowState_pub = n.advertise<laikago_msgs::LowState>("/laikago_gazebo/lowState/state", 1);
    servo_pub[0] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FR_hip_controller/command", 1);
    servo_pub[1] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FR_thigh_controller/command", 1);
    servo_pub[2] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FR_calf_controller/command", 1);
    servo_pub[3] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FL_hip_controller/command", 1);
    servo_pub[4] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FL_thigh_controller/command", 1);
    servo_pub[5] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FL_calf_controller/command", 1);
    servo_pub[6] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RR_hip_controller/command", 1);
    servo_pub[7] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RR_thigh_controller/command", 1);
    servo_pub[8] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RR_calf_controller/command", 1);
    servo_pub[9] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RL_hip_controller/command", 1);
    servo_pub[10] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RL_thigh_controller/command", 1);
    servo_pub[11] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RL_calf_controller/command", 1);
    
    std::cout << "stand up" << std::endl;
    double dt = 0.001;
    Quadruped quad ;
    quad.setQuadruped(1); // 1 for Aliengo, 2 for A1
    // initialize new leg controller and state estimate object
    StateEstimate stateEstimate;
    std::cout << "start Leg Controller" << std::endl;
    LegController* legController = new LegController(quad);

    std::cout << "start state estimate" << std::endl;
    StateEstimatorContainer* stateEstimator = new StateEstimatorContainer(
                       &lowState.imu, legController->data,&stateEstimate);
    stateEstimator->addEstimator<ContactEstimator>();
    stateEstimator->addEstimator<CheaterOrientationEstimator>();
    stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();
    // change to pybullet (not implemented orientation yet)
    //stateEstimator->addEstimator<CheaterOrientationEstimator_Pybullet>();
    //stateEstimator->addEstimator<CheaterPositionVelocityEstimator_Pybullet>();
    // stateEstimator->addEstimator<CheaterPybulletEstimator>();


    // initialize new Gait scheduler object
    std::cout << "start gait" << std::endl;
    GaitScheduler<double>* gaitScheduler = new GaitScheduler<double>(dt);
    gaitScheduler->initialize();

    // initialize desired command
    std::cout << "start cmd" << std::endl;
    teleCmd* _tele = new teleCmd();
    DesiredStateCommand* desiredStateCommand = new DesiredStateCommand(_tele, &stateEstimate, dt);
    
    // initialize FSM
    std::cout << "start controlFSMData" << std::endl;
    ControlFSMData* _controlData = new ControlFSMData;
    _controlData->_quadruped = &quad;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_gaitScheduler = gaitScheduler;
    _controlData->_legController = legController;
    _controlData->_desiredStateCommand = desiredStateCommand;

    std::cout << "start fsm and init" << std::endl;
    FSM_State* fsm = new FSM_State(_controlData);
    motion_init();
    //enableTorqueMode(true);

    //motion_init();
    std::cout << "FSM_Started" << std::endl;
    
    legController->updateData();
    stateEstimator->run();

    //fsm->runLeastSquare();
    //fsm->QPstand(); // MPC locomotion implemented in this function
    //fsm->PDstand(); // standup with PD control

    /****************************************************************************
    This is the PDStand function above, but with the lowState publish state
    /****************************************************************************/
    /*
    int counter = 0;
    // initial foot possition
    Mat34<double> init_foot_pos;
    _controlData->_legController->updateData();
    for(int i = 0; i < 4; i++){
        init_foot_pos.col(i) = _controlData->_legController->data[i].p;
    }
    double h = 0.4; // standup height
    double side_sign[4] = {-1, 1, -1, 1};
    Vec3<double> ff_force_world(0, 0, 0);

    while(ros::ok()){
        double progress = counter * 0.001;
        if(progress > 1.)  {progress = 1.;}

        _controlData->_legController->updateData();  
        _controlData->_stateEstimator->run();    

        for(int i = 0; i < 4; i++){
            _controlData->_legController->commands[i].kpCartesian = Vec3<double>(400,400,900).asDiagonal();
            _controlData->_legController->commands[i].kdCartesian = Vec3<double>(20,20,20).asDiagonal();
            _controlData->_legController->commands[i].pDes << 0, side_sign[i] * 0.083, 0;
            _controlData->_legController->commands[i].pDes[2] = -h*progress + (1. - progress) * init_foot_pos(2, i);
            // _data->_legController->commands[i].feedforwardForce = _data->_stateEstimator->getResult().rBody.transpose() * ff_force_world;
            // std::cout << "leg  " << i << "  " << _data->_legController->data[i].p << std::endl;
        }
        _controlData->_legController->updateCommand();
        counter++;
        lowState_pub.publish(lowState);
        loop_rate.sleep();
    }
    */
    /****************************************************************************
    This is the balance controller 
    /****************************************************************************/
    BalanceController balanceController;
     // reset forces and steps to 0
    Mat34<double>footFeedForwardForces = Mat34<double>::Zero();
    Mat34<double>footstepLocations = Mat34<double>::Zero();
    double minForce = 5;
    double maxForce = 500;
    double contactStateScheduled[4] = {1, 1, 1, 1};
    double minForces[4] = {minForce, minForce, minForce, minForce};
    double maxForces[4] = {maxForce, maxForce, maxForce, maxForce};

    //double COM_weights_stance[3] = {50, 50, 50};
    //double Base_weights_stance[3] = {500, 150, 30};// for QP locomotion
    double COM_weights_stance[3] = {5, 5, 10};
    double Base_weights_stance[3] = {10, 10, 20};
    double pFeet[12], p_act[3], v_act[3], O_err[3], rpy[3],v_des[3], p_des[3],omegaDes[3];
    double se_xfb[13];
    double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];
    double b_control[6];
    int counter = 0;
    _controlData->_legController->updateData();
    _controlData->_stateEstimator->run();
    // se_xfb[3] = 1.0; 
    //  int during = 10;
    // position & rpy desired
    for(int i = 0; i < 3; i++){
        p_des[i] = _controlData->_stateEstimator->getResult().position(i); 
        v_des[i] = 0;
        omegaDes[i] = 0;
        rpy[i] = 0;
    }
    rpy[2] = _controlData->_stateEstimator->getResult().rpy(2);
    p_des[2] = 0.3;

    while(ros::ok()) {
        _controlData->_legController->updateData();
        _controlData->_stateEstimator->run();

        for (int i = 0; i < 4; i++) {
            se_xfb[i] = _controlData->_stateEstimator->getResult().orientation(i);
        }

        for (int i = 0; i < 3; i++) {
            //rpy[i] = 0;
            //rpy[i] = _data->_stateEstimator->getResult().rpy(i);
            //p_des[i] = _data->_stateEstimator->getResult().position(i);
            p_act[i] = _controlData->_stateEstimator->getResult().position(i);
            v_act[i] = _controlData->_stateEstimator->getResult().vBody(i);
            // v_des[i] = 0;
            //v_des[2] = 0.005;

            se_xfb[4 + i] = _controlData->_stateEstimator->getResult().position(i);
            se_xfb[7 + i] = _controlData->_stateEstimator->getResult().omegaBody(i);
            se_xfb[10 + i] = _controlData->_stateEstimator->getResult().vBody(i);

            // Set the translational and orientation gains
            kpCOM[i] = 30;   //_data->controlParameters->kpCOM(i);
            kdCOM[i] = 10;     //_data->controlParameters->kdCOM(i);
            kpBase[i] = 80;     //_data->controlParameters->kpBase(i);
            kdBase[i] = 50; //  _data->controlParameters->kdBase(i);
        }

        kpCOM[2] = 50;
        //kdCOM[2] = 10;

        //Vec3<double> pFeetVec;
        Vec3<double> pFeetVecCOM;

        // Get the foot locations relative to COM
        for (int leg = 0; leg < 4; leg++) {
            // computeLegJacobianAndPosition(&_data->_quadruped, _data->_legController->data[leg].q,
            //                           (Mat3<double>*)nullptr, &pFeetVec, leg);
            //pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                      //(_data->_quadruped->getHipLocation(leg) + pFeetVec);

            pFeetVecCOM =  _controlData->_stateEstimator->getResult().rBody.transpose() *
                    (_controlData->_quadruped->getHipLocation(leg) + _controlData->_legController->data[leg].p);


            pFeet[leg * 3] = pFeetVecCOM[0];
            pFeet[leg * 3 + 1] = pFeetVecCOM[1];
            pFeet[leg * 3 + 2] = pFeetVecCOM[2];
            //std::cout << "pFeet" << leg << std::endl;
        }

        // balance controller options
        balanceController.set_alpha_control(0.01);
        balanceController.set_friction(0.6);
        balanceController.set_mass(_controlData->_quadruped->mass);
        balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
        balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
        balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
        balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
        balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                                      O_err, _controlData->_stateEstimator->getResult().rpy(2));
        // balanceController.print_QPData();
        double fOpt[12];
        balanceController.solveQP_nonThreaded(fOpt);

        for (int leg = 0; leg < 4; leg++) {
            footFeedForwardForces.col(leg) << fOpt[leg * 3], fOpt[leg * 3 + 1],
            fOpt[leg * 3 + 2]; // force in world frame, need to convert to body frame

            _controlData->_legController->commands[leg].feedforwardForce = _controlData->_stateEstimator->getResult().rBody.transpose() *
                                                                    footFeedForwardForces.col(leg);
            //_data->_stateEstimator->getResult().rBody.transpose() * footFeedForwardForces.col(leg); 
           // QP << _data->_legController->commands[leg].feedforwardForce[2] << " ";
        }
        _controlData->_legController->updateCommand();
        loop_rate.sleep();
    
    }
    //std::cout << "finished" << std::endl;
    //delete quad;

    delete desiredStateCommand;
    delete gaitScheduler;
    delete legController;
    delete stateEstimator;
    delete _tele;
    delete _controlData;
    delete fsm;

    return 0;
}
