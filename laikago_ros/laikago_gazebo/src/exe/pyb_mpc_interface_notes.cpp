/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/
/************************************************************************
NOTES:
Notes when trying to run together between gazebo and pybullet simulations.

Working interface for using mpc is: pyb_mpc.cpp


--------------------------------------------------------------------------
This file: receive data from pybullet, call MPC, send back to pybullet.
Subscribe:
- pybulletState
Publish:
- lowState (full robot state)
- legCmds (desired leg commands, i.e. pDes, vDes, feedforwardForce, kpCartesian, kdCartesian)

These are read and visualized in pybullet, see run_ros_mpc.py

pyb_send_state.cc       # send ROS states/commands to pybullet
pyb_mpc_interface.cc    # receive robot state from pybullet, run MPC, send result back

Notes: see gazebo_msgs
- pause/unpause sim
- get/set model/link states

need to update legController data -> lowState motorStates
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


class PybulletStateListener
{
public:
    PybulletStateListener(){
        pybulletState_sub = nm.subscribe("/laikago_gazebo/pybulletState", 1, &PybulletStateListener::PybStateCallback, this);
        servo_sub[0] = nm.subscribe("/laikago_gazebo/FR_hip_controller/pybstate", 1, &PybulletStateListener::FRhipCallback, this);
        servo_sub[1] = nm.subscribe("/laikago_gazebo/FR_thigh_controller/pybstate", 1, &PybulletStateListener::FRthighCallback, this);
        servo_sub[2] = nm.subscribe("/laikago_gazebo/FR_calf_controller/pybstate", 1, &PybulletStateListener::FRcalfCallback, this);
        servo_sub[3] = nm.subscribe("/laikago_gazebo/FL_hip_controller/pybstate", 1, &PybulletStateListener::FLhipCallback, this);
        servo_sub[4] = nm.subscribe("/laikago_gazebo/FL_thigh_controller/pybstate", 1, &PybulletStateListener::FLthighCallback, this);
        servo_sub[5] = nm.subscribe("/laikago_gazebo/FL_calf_controller/pybstate", 1, &PybulletStateListener::FLcalfCallback, this);
        servo_sub[6] = nm.subscribe("/laikago_gazebo/RR_hip_controller/pybstate", 1, &PybulletStateListener::RRhipCallback, this);
        servo_sub[7] = nm.subscribe("/laikago_gazebo/RR_thigh_controller/pybstate", 1, &PybulletStateListener::RRthighCallback, this);
        servo_sub[8] = nm.subscribe("/laikago_gazebo/RR_calf_controller/pybstate", 1, &PybulletStateListener::RRcalfCallback, this);
        servo_sub[9] = nm.subscribe("/laikago_gazebo/RL_hip_controller/pybstate", 1, &PybulletStateListener::RLhipCallback, this);
        servo_sub[10] = nm.subscribe("/laikago_gazebo/RL_thigh_controller/pybstate", 1, &PybulletStateListener::RLthighCallback, this);
        servo_sub[11] = nm.subscribe("/laikago_gazebo/RL_calf_controller/pybstate", 1, &PybulletStateListener::RLcalfCallback, this);
    }

    void FRhipCallback(const laikago_msgs::MotorState& msg)
    {
        //start_up = false;

        lowState.motorState[0].mode = msg.mode;
        lowState.motorState[0].position = msg.position;
        lowState.motorState[0].velocity = msg.velocity;
        lowState.motorState[0].torque = msg.torque;
        std::cout << "called FRhip, new pos: " << lowState.motorState[0].position << "vel: " << lowState.motorState[0].velocity << std::endl;
    }

    void FRthighCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[1].mode = msg.mode;
        lowState.motorState[1].position = msg.position;
        lowState.motorState[1].velocity = msg.velocity;
        lowState.motorState[1].torque = msg.torque;
    }

    void FRcalfCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[2].mode = msg.mode;
        lowState.motorState[2].position = msg.position;
        lowState.motorState[2].velocity = msg.velocity;
        lowState.motorState[2].torque = msg.torque;
    }

    void FLhipCallback(const laikago_msgs::MotorState& msg)
    {
        //start_up = false;
        lowState.motorState[3].mode = msg.mode;
        lowState.motorState[3].position = msg.position;
        lowState.motorState[3].velocity = msg.velocity;
        lowState.motorState[3].torque = msg.torque;
    }

    void FLthighCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[4].mode = msg.mode;
        lowState.motorState[4].position = msg.position;
        lowState.motorState[4].velocity = msg.velocity;
        lowState.motorState[4].torque = msg.torque;
    }

    void FLcalfCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[5].mode = msg.mode;
        lowState.motorState[5].position = msg.position;
        lowState.motorState[5].velocity = msg.velocity;
        lowState.motorState[5].torque = msg.torque;
    }

    void RRhipCallback(const laikago_msgs::MotorState& msg)
    {
        //start_up = false;
        lowState.motorState[6].mode = msg.mode;
        lowState.motorState[6].position = msg.position;
        lowState.motorState[6].velocity = msg.velocity;
        lowState.motorState[6].torque = msg.torque;
    }

    void RRthighCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[7].mode = msg.mode;
        lowState.motorState[7].position = msg.position;
        lowState.motorState[7].velocity = msg.velocity;
        lowState.motorState[7].torque = msg.torque;
    }

    void RRcalfCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[8].mode = msg.mode;
        lowState.motorState[8].position = msg.position;
        lowState.motorState[8].velocity = msg.velocity;
        lowState.motorState[8].torque = msg.torque;
    }

    void RLhipCallback(const laikago_msgs::MotorState& msg)
    {
        //start_up = false;
        lowState.motorState[9].mode = msg.mode;
        lowState.motorState[9].position = msg.position;
        lowState.motorState[9].velocity = msg.velocity;
        lowState.motorState[9].torque = msg.torque;
    }

    void RLthighCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[10].mode = msg.mode;
        lowState.motorState[10].position = msg.position;
        lowState.motorState[10].velocity = msg.velocity;
        lowState.motorState[10].torque = msg.torque;
    }

    void RLcalfCallback(const laikago_msgs::MotorState& msg)
    {
        lowState.motorState[11].mode = msg.mode;
        lowState.motorState[11].position = msg.position;
        lowState.motorState[11].velocity = msg.velocity;
        lowState.motorState[11].torque = msg.torque;
    }

    void PybStateCallback(const laikago_msgs::PybulletState& msg)
    {
        /* Set following information:
        struct StateEstimate {
            Vec3<double> position;
            Quat<double> orientation;
            Vec3<double> rpy;
            RotMat<double> rBody;
            Vec3<double> vBody;
            Vec3<double> omegaBody;
            Vec3<double> vWorld;
            Vec3<double> omegaWorld;
            
            Vec4<double> contactEstimate;
            Vec3<double> aBody, aWorld;

            // adding
            Vec3<double> v_des
            double yaw_rate
        }
        */

        // quick hack, should be with Eigen mapping
        for (int i = 0; i < 3; i++){
            // State Estimate
            result.position[i] = msg.position[i];
            result.rpy[i] = msg.rpy[i];
            result.vBody[i] = msg.vBody[i];
            result.omegaBody[i] = msg.omegaBody[i];
            result.vWorld[i] = msg.vWorld[i];
            result.omegaWorld[i] = msg.omegaWorld[i];
            // v_des
            v_des[i] = msg.v_des[i];

            // also set extras not currently used
            result.aBody[i] = 0;
            result.aWorld[i] = 0;

        }

        for (int i = 0; i < 4; i++){
            result.orientation[i] = msg.orientation[i];
        }
        int rBodyIndex = 0;
        for (int i = 0; i < 3; i++){
            for (int j = 0; j < 3; j++){
                result.rBody(i,j) = msg.rBody[rBodyIndex];
                rBodyIndex++;
            }
        }
        yaw_rate = msg.yaw_rate_des;
        hasData = 1;

    }
    int hasData = 0;
    Vec3<double> v_des;
    double yaw_rate;
    StateEstimate result;

private:
    ros::NodeHandle nm;
    ros::Subscriber pybulletState_sub, servo_sub[12]; // footForce_sub[4], imu_sub;
    
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "laikago_gazebo_servo");
    ros::Publisher lowCmd_pub, highCmd_pub; 
    ros::NodeHandle n;
    ros::Publisher lowState_pub; // full robot state
    ros::Publisher legCmds_pub, pybState_pub;  // desired leg commands from stand/MPC
   
    ros::AsyncSpinner spinner(1); // one threads
    ros::Rate loop_rate(1000);
    spinner.start();

    // lowState must be updated with pybullet data
    //CurrentState listen_publish_obj;

    usleep(300000); // must wait 300ms, to get first state
   
    laikago_msgs::LegCmdArray legCmdArray;
    laikago_msgs::PybulletState ros2pybstate;
    PybulletStateListener pyb_listen_obj;

    // the following nodes have been initialized by "gazebo.launch"
    lowState_pub = n.advertise<laikago_msgs::LowState>("/laikago_gazebo/lowState/state", 1);
    legCmds_pub = n.advertise<laikago_msgs::LegCmdArray>("/laikago_gazebo/legCmds", 1);
    pybState_pub = n.advertise<laikago_msgs::PybulletState>("/laikago_gazebo/ros2pybstate", 1);
    // servo_pub[0] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FR_hip_controller/command", 1);
    // servo_pub[1] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FR_thigh_controller/command", 1);
    // servo_pub[2] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FR_calf_controller/command", 1);
    // servo_pub[3] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FL_hip_controller/command", 1);
    // servo_pub[4] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FL_thigh_controller/command", 1);
    // servo_pub[5] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/FL_calf_controller/command", 1);
    // servo_pub[6] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RR_hip_controller/command", 1);
    // servo_pub[7] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RR_thigh_controller/command", 1);
    // servo_pub[8] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RR_calf_controller/command", 1);
    // servo_pub[9] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RL_hip_controller/command", 1);
    // servo_pub[10] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RL_thigh_controller/command", 1);
    // servo_pub[11] = n.advertise<laikago_msgs::MotorCmd>("/laikago_gazebo/RL_calf_controller/command", 1);
    
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
    //stateEstimator->addEstimator<ContactEstimator>();
    //stateEstimator->addEstimator<CheaterOrientationEstimator>();
    //stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();
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
    //motion_init();
    //enableTorqueMode(true);

    //motion_init();
    std::cout << "FSM_Started" << std::endl;
    
    legController->updateData();
    //stateEstimator->run();

    //fsm->runLeastSquare();
    //fsm->QPstand(); // MPC locomotion implemented in this function
    //fsm->PDstand(); // standup with PD control


    /******************************************

    Have loop where can delete and reset, MPC should be re-init upon environment reset
    **********************************/

    // wait to get data from pybullet
    while (!pyb_listen_obj.hasData){
        usleep(300000); 
        std::cout << "waiting on pybullet data " << pyb_listen_obj.hasData << std::endl;
    }
    std::cout << "now have data " << std::endl;
    /*======================== MPC locomotion ====================*/
    // define CMPC
    //ConvexMPCLocomotion Cmpc(0.001, 30);
    ConvexMPCLocomotion Cmpc(0.001, 30);
    int counter = 0;
    // replace state estimator with data from pybullet, call MPC
    while(ros::ok()) {
        // this updates leg q,qd,tau from motorState 
        _controlData->_legController->updateData();
        // replace this with the data from pybullet
        _controlData->_stateEstimator->setStateEstimate(pyb_listen_obj.result);

        // set desired body vel (v_des) and yaw_rate from pyb_listen_obj as well
        _controlData->_desiredStateCommand->setStateCommands(pyb_listen_obj.v_des, pyb_listen_obj.yaw_rate);

        // set gait (trot)
        Cmpc.setGaitNum(2);
        // run MPC
        Cmpc.run(*_controlData);

        // [BUG] Does not work without this..
        usleep(10000); 

        // publish robot state (will not be accurate between simulations?)
        //lowState_pub.publish(lowState);
        // send control to robot (not needed for this ..)
        _controlData->_legController->updateCommandNoSend();

        // loop through legController commands to create LegCmdArray message 
        for (int i = 0; i < 4; i++){
            for (int j = 0; j<3; j++){
                legCmdArray.legCmds[i].pDes[j] = _controlData->_legController->commands[i].pDes[j];
                legCmdArray.legCmds[i].vDes[j] = _controlData->_legController->commands[i].vDes[j];
                legCmdArray.legCmds[i].kpCartesianDiag[j] = _controlData->_legController->commands[i].kpCartesian(j,j);
                legCmdArray.legCmds[i].kdCartesianDiag[j] = _controlData->_legController->commands[i].kdCartesian(j,j);
                legCmdArray.legCmds[i].feedforwardForce[j] = _controlData->_legController->commands[i].feedforwardForce[j];
                legCmdArray.motorCmd[i*3+j] = lowCmd.motorCmd[i*3+j].torque;
                std::cout <<  lowCmd.motorCmd[i*3+j].torque << std::endl;
            }
        }
        // publish for pybullet
        legCmds_pub.publish(legCmdArray);



        /********************************************************
        Send robot state to pybullet for comparison
        *********************************************************/
        StateEstimate rosState = _controlData->_stateEstimator->getResult();
        for (int i = 0; i < 3; i++){
            // State Estimate
            ros2pybstate.position[i] = rosState.position[i];
            ros2pybstate.rpy[i] = rosState.rpy[i];
            ros2pybstate.vBody[i] = rosState.vBody[i];
            ros2pybstate.omegaBody[i] = rosState.omegaBody[i];
            ros2pybstate.vWorld[i] = rosState.vWorld[i];
            ros2pybstate.omegaWorld[i] = rosState.omegaWorld[i];
        }
        for (int i = 0; i < 4; i++){
            ros2pybstate.orientation[i] = rosState.orientation[i];
        }
        int rBodyIndex = 0;
        for (int i = 0; i < 3; i++){
            for (int j = 0; j < 3; j++){
                ros2pybstate.rBody[rBodyIndex] = rosState.rBody(i,j);
                rBodyIndex++;
            }
        }
        pybState_pub.publish(ros2pybstate);
        /********************************************************
        done sending
        *********************************************************/
        loop_rate.sleep();
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
