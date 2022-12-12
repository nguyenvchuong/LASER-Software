/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/
/************************************************************************

This file: run run PDStand(), QPStand();  MPC as in servo.cpp example
Publish:
- lowState (full robot state)
- legCmds (desired leg commands, i.e. pDes, vDes, feedforwardForce, kpCartesian, kdCartesian)

These are read and visualized in pybullet, see usc_learning/ros_interface/example_vis_ros_state.py


pyb_send_state.cc       # send ROS states/commands to pybullet (this one)
pyb_mpc_interface.cc    # receive robot state from pybullet, run MPC, send result back

Notes: see gazebo_msgs
- pause/unpause sim
- get/set model/link states

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
#include "../../ConvexMPC/ConvexMPCLocomotion.h"
#include "laikago_msgs/LegCmdArray.h"
#include "laikago_msgs/LegCmd.h"
#include "laikago_msgs/PybulletState.h"


using namespace std;
using namespace laikago_model;


int main(int argc, char **argv)
{
    ros::init(argc, argv, "laikago_gazebo_servo");
    ros::Publisher lowCmd_pub, highCmd_pub; //for rviz visualization
    ros::NodeHandle n;
    ros::Publisher lowState_pub; // full robot state
    ros::Publisher legCmds_pub, pybState_pub;  // desired leg commands from stand/MPC
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
    ros::Rate mpc_loop_rate(1000);
    spinner.start();

    CurrentState listen_publish_obj;

    usleep(300000); // must wait 300ms, to get first state
   
    laikago_msgs::LegCmdArray legCmdArray;
    laikago_msgs::PybulletState ros2pybstate;
    

    // the following nodes have been initialized by "gazebo.launch"
    lowState_pub = n.advertise<laikago_msgs::LowState>("/laikago_gazebo/lowState/state", 1);
    legCmds_pub = n.advertise<laikago_msgs::LegCmdArray>("/laikago_gazebo/legCmds", 1);
    pybState_pub = n.advertise<laikago_msgs::PybulletState>("/laikago_gazebo/ros2pybstate", 1);
    // needed to update commands
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
    quad.setQuadruped(2); // 1 for Aliengo, 2 for A1
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
    // change to pybullet (not implemented orientation yet) [doesn't seem necessary? can just replace stateEstimator data?]
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

    This is the fsm->PDStand function above, but with the lowState publish state.
    Run at first to stand, then MPC

    /****************************************************************************/

    int counter = 0;
  
    // initial foot position
    Mat34<double> init_foot_pos;
    _controlData->_legController->updateData();
    for(int i = 0; i < 4; i++){
        init_foot_pos.col(i) = _controlData->_legController->data[i].p;
    }
    double h = 0.3; // standup height
    double side_sign[4] = {-1, 1, -1, 1};
    Vec3<double> ff_force_world(0, 0, 0);

    while(ros::ok() && counter < 1000){
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

        
        // loop through legController commands to create LegCmdArray message 
        for (int i = 0; i < 4; i++){
            for (int j = 0; j<3; j++){
                legCmdArray.legCmds[i].pDes[j] = _controlData->_legController->commands[i].pDes[j];
                legCmdArray.legCmds[i].vDes[j] = _controlData->_legController->commands[i].vDes[j];
                // legCmdArray.legCmds[i].kpCartesianDiag[j] = _controlData->_legController->commands[i].kpCartesian.diagonal()[j];
                // legCmdArray.legCmds[i].kdCartesianDiag[j] = _controlData->_legController->commands[i].kdCartesian.diagonal()[j];
                legCmdArray.legCmds[i].kpCartesianDiag[j] = _controlData->_legController->commands[i].kpCartesian(j,j);
                legCmdArray.legCmds[i].kdCartesianDiag[j] = _controlData->_legController->commands[i].kdCartesian(j,j);
                legCmdArray.legCmds[i].feedforwardForce[j] = _controlData->_legController->commands[i].feedforwardForce[j];
            }
        }
        // publish for pybullet
        legCmds_pub.publish(legCmdArray);

        loop_rate.sleep();
    }

    /****************************************************************************

    Run MPC like in FSM.cpp, gradually increasing forward speed. See fsm->QPstand()

    /****************************************************************************/

    // define CMPC
    ConvexMPCLocomotion Cmpc(0.001, 30);
    counter = 0;
    int call_mpc_frequency = 1; //1 is call every time 
    /*======================== MPC locomotion ====================*/
    while(ros::ok() ){ //&& counter < 1000
        _controlData->_legController->updateData();
        _controlData->_stateEstimator->run();
        Vec3<double> v_des(0, 0, 0);
        double yaw_rate = 0;
        Cmpc.setGaitNum(2);
        if(fabs(_controlData->_stateEstimator->getResult().rpy[2]) > 0.01){
            if(_controlData->_stateEstimator->getResult().rpy[2] < 0)
                yaw_rate = 0.5;
            else yaw_rate = -0.5;
            _controlData->_desiredStateCommand->setStateCommands(v_des, yaw_rate);
        }

        if(counter > 3000){
            v_des[0] = 0.5;
            // v_des[1] = 0;
            //yaw_rate = 0.1;
        }
        if(counter > 6000){
            // Cmpc.setGaitNum(6);
            v_des[0] = 1;
            //v_des[1] = 0.2;
            //yaw_rate = 0;   
        }
        if(counter > 12000){
            v_des[0] = 1.5;
        }
        if(counter > 15000){
            v_des[0] = 2;
            // yaw_rate = 0;
        }
        _controlData->_desiredStateCommand->setStateCommands(v_des, yaw_rate);
        // if(counter > 3000){
        //   QP << _data->_desiredStateCommand->data.stateDes(5) << " ";
        //   QP << _data->_stateEstimator->getResult().rpy[2] << "\n";
        // }

        // run MPC, only at MPC frequency 
        if (counter % call_mpc_frequency == 0){
            // run MPC
            Cmpc.run(*_controlData);
        }
        
        // publish robot state
        //lowState_pub.publish(lowState);
        // send control to robot
        _controlData->_legController->updateCommand();
        // for (int i=0; i<2; i++){
        //     _controlData->_legController->updateCommand();
        //     loop_rate.sleep();
        // }

        // loop through legController commands to create LegCmdArray message 
        // for (int i = 0; i < 4; i++){
        //     for (int j = 0; j<3; j++){
        //         legCmdArray.legCmds[i].pDes[j] = _controlData->_legController->commands[i].pDes[j];
        //         legCmdArray.legCmds[i].vDes[j] = _controlData->_legController->commands[i].vDes[j];
        //         legCmdArray.legCmds[i].kpCartesianDiag[j] = _controlData->_legController->commands[i].kpCartesian(j,j);
        //         legCmdArray.legCmds[i].kdCartesianDiag[j] = _controlData->_legController->commands[i].kdCartesian(j,j);
        //         legCmdArray.legCmds[i].feedforwardForce[j] = _controlData->_legController->commands[i].feedforwardForce[j];
        //     }
        // }
        // publish for pybullet
        //legCmds_pub.publish(legCmdArray);

        /********************************************************
        Send robot state to pybullet for comparison
        *********************************************************/
        // StateEstimate rosState = _controlData->_stateEstimator->getResult();
        // for (int i = 0; i < 3; i++){
        //     // State Estimate
        //     ros2pybstate.position[i] = rosState.position[i];
        //     ros2pybstate.rpy[i] = rosState.rpy[i];
        //     ros2pybstate.vBody[i] = rosState.vBody[i];
        //     ros2pybstate.omegaBody[i] = rosState.omegaBody[i];
        //     ros2pybstate.vWorld[i] = rosState.vWorld[i];
        //     ros2pybstate.omegaWorld[i] = rosState.omegaWorld[i];
        // }
        // for (int i = 0; i < 4; i++){
        //     ros2pybstate.orientation[i] = rosState.orientation[i];
        // }
        // int rBodyIndex = 0;
        // for (int i = 0; i < 3; i++){
        //     for (int j = 0; j < 3; j++){
        //         ros2pybstate.rBody[rBodyIndex] = rosState.rBody(i,j);
        //         rBodyIndex++;
        //     }
        // }
        // pybState_pub.publish(ros2pybstate);
        /********************************************************
        done sending
        *********************************************************/


        loop_rate.sleep();
        //mpc_loop_rate.sleep();
        counter++;

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
