/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/
/************************************************************************

This file: receive data from pybullet, call MPC, send back to pybullet.
Service:
- MPCService, with PybulletFullState Request, and LegCmdArray Response
Subscribe:
- reset MPC (firstRun = true)
- set new Gait num

This is a service called from pybullet, no gazebo updates
- see usc_learning/ros_interface/run_mpc.py


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
#include "laikago_msgs/LegCmdArray.h"
#include "laikago_msgs/LegCmd.h"
#include "laikago_msgs/PybulletState.h"
#include "laikago_msgs/PybulletFullState.h"
#include "laikago_msgs/MPCService.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
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

using namespace std;
using namespace laikago_model;

class PybulletStateListener
{
public:
    PybulletStateListener() : Cmpc(0.001, 30) {
        //pybulletState_sub = nm.subscribe("/laikago_gazebo/pybulletFullState", 1, &PybulletStateListener::PybStateCallback, this);
        mpc_service = nm.advertiseService("/laikago_gazebo/mpc_comm", &PybulletStateListener::CallMPC, this); //persistent, true?
        mpc_reset_sub = nm.subscribe("/laikago_gazebo/reset_mpc", 1, &PybulletStateListener::ResetMPC, this);
        gait_sub = nm.subscribe("/laikago_gazebo/set_gait", 1, &PybulletStateListener::SetGait, this);

    }

    void ResetMPC(const std_msgs::Empty& msg)
    {
        Cmpc.reset(); // verify this is enough to reset
        std::cout << "reset MPC" << std::endl;
    }
    void SetGait(const std_msgs::Int32& msg)
    {
        gaitNum = msg.data;
        Cmpc.setGaitNum(gaitNum);
    }

    /*
    TODO: add option for calling balance controller as well
    */

    // formats: (laikago_msgs::PybulletFullState& msg, laikago_msgs::LegCmdArray& response)
    bool CallMPC(laikago_msgs::MPCServiceRequest& msg, laikago_msgs::MPCServiceResponse& response)
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
        }
        MotorStates: lowState.motorState[].{pos/vel/torque}
        // desired body vel and yaw_rate
        Vec3<double> v_des
        double yaw_rate
        */
        /**********************************************************************************
        Read in current pybullet state
        ***********************************************************************************/
        // quick hack, should be with Eigen mapping
        for (int i = 0; i < 3; i++){
            result.position[i] = msg.pybState.position[i];
            result.rpy[i] = msg.pybState.rpy[i];
            result.vBody[i] = msg.pybState.vBody[i];
            result.omegaBody[i] = msg.pybState.omegaBody[i];
            result.vWorld[i] = msg.pybState.vWorld[i];
            result.omegaWorld[i] = msg.pybState.omegaWorld[i];
            // v_des
            v_des[i] = msg.pybState.v_des[i];
            // also set extras not currently used
            result.aBody[i] = 0;
            result.aWorld[i] = 0;
        }
        // orientation
        for (int i = 0; i < 4; i++){
            result.orientation[i] = msg.pybState.orientation[i];
        }
        // rotation matrix (note, column major order for eigen )
        int rBodyIndex = 0;
        for (int i = 0; i < 3; i++){
            for (int j = 0; j < 3; j++){
                result.rBody(i,j) = msg.pybState.rBody[rBodyIndex];
                rBodyIndex++;
            }
        }
        // loop through motor states
        for (int i =0; i < 12; i++){
            lowState.motorState[i].mode = msg.pybState.motorState[i].mode;
            lowState.motorState[i].position = msg.pybState.motorState[i].position;
            lowState.motorState[i].velocity = msg.pybState.motorState[i].velocity;
            lowState.motorState[i].torque = msg.pybState.motorState[i].torque;
        }
        yaw_rate = msg.pybState.yaw_rate_des;
        hasData = 1;

        /**********************************************************************************
        Set MPC conditions from pybullet data received, run, and set leg commands response.
        ***********************************************************************************/

        // this updates leg q,qd,tau from motorState 
        _controlData->_legController->updateData();
        // replace this with the data from pybullet
        _controlData->_stateEstimator->setStateEstimate(result);
        // set desired body vel (v_des) and yaw_rate
        _controlData->_desiredStateCommand->setStateCommands(v_des, yaw_rate);

        // set gait (trot)
        Cmpc.setGaitNum(gaitNum);
        // run MPC
        Cmpc.run(*_controlData);

        // publish robot state (if want to visualize)
        //lowState_pub.publish(lowState);
        // updates legController commands, but does not send control to robot
        _controlData->_legController->updateCommandNoSend();

        // loop through legController commands to create LegCmdArray message 
        for (int i = 0; i < 4; i++){
            for (int j = 0; j<3; j++){
                response.legCmdArray.legCmds[i].pDes[j] = _controlData->_legController->commands[i].pDes[j];
                response.legCmdArray.legCmds[i].vDes[j] = _controlData->_legController->commands[i].vDes[j];
                response.legCmdArray.legCmds[i].kpCartesianDiag[j] = _controlData->_legController->commands[i].kpCartesian(j,j);
                response.legCmdArray.legCmds[i].kdCartesianDiag[j] = _controlData->_legController->commands[i].kdCartesian(j,j);
                response.legCmdArray.legCmds[i].feedforwardForce[j] = _controlData->_legController->commands[i].feedforwardForce[j];
                response.legCmdArray.motorCmd[i*3+j] = lowCmd.motorCmd[i*3+j].torque;
            }
        }
        // publish for pybullet (see other file)
        //legCmds_pub.publish(legCmdArray);

        return true;

    }
    int hasData = 0;
    Vec3<double> v_des;
    double yaw_rate;
    StateEstimate result;
    ControlFSMData* _controlData;
    ConvexMPCLocomotion Cmpc;

private:
    ros::NodeHandle nm;
    ros::Subscriber pybulletState_sub; 
    ros::Subscriber mpc_reset_sub, gait_sub;
    ros::ServiceServer mpc_service;
    laikago_msgs::LegCmdArray legCmdArray;
    int gaitNum = 2;
    
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_mpc_interface");
    ros::Publisher lowCmd_pub, highCmd_pub; 
    ros::NodeHandle n;
    //ros::Publisher lowState_pub; // full robot state
    //ros::Publisher legCmds_pub, pybState_pub;  // desired leg commands from stand/MPC
   
    ros::AsyncSpinner spinner(1); // one threads
    ros::Rate loop_rate(1000);
    spinner.start();

    // lowState must be updated with pybullet data
    //CurrentState listen_publish_obj;

    usleep(300000); // must wait 300ms, to get first state
   
    //laikago_msgs::LegCmdArray legCmdArray;
    //laikago_msgs::PybulletState ros2pybstate;

    // the following nodes have been initialized by "gazebo.launch"
    //lowState_pub = n.advertise<laikago_msgs::LowState>("/laikago_gazebo/lowState/state", 1);
    //legCmds_pub = n.advertise<laikago_msgs::LegCmdArray>("/laikago_gazebo/legCmds", 1);
    //pybState_pub = n.advertise<laikago_msgs::PybulletState>("/laikago_gazebo/ros2pybstate", 1);

    
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


    // initialize new Gait scheduler object
    std::cout << "start gait" << std::endl;
    GaitScheduler<double>* gaitScheduler = new GaitScheduler<double>(dt);
    gaitScheduler->initialize();

    // initialize desired command
    std::cout << "start cmd" << std::endl;
    teleCmd* _tele = new teleCmd();
    DesiredStateCommand* desiredStateCommand = new DesiredStateCommand(_tele, &stateEstimate, dt);

    // initialize FSMData
    std::cout << "start controlFSMData" << std::endl;
    ControlFSMData* _controlData = new ControlFSMData;
    _controlData->_quadruped = &quad;
    _controlData->_stateEstimator = stateEstimator;
    _controlData->_gaitScheduler = gaitScheduler;
    _controlData->_legController = legController;
    _controlData->_desiredStateCommand = desiredStateCommand;

    std::cout << "start fsm and init" << std::endl;
    FSM_State* fsm = new FSM_State(_controlData);
    std::cout << "FSM_Started" << std::endl;
    
    // Start interface obj
    PybulletStateListener pyb_listen_obj; 
    pyb_listen_obj._controlData = _controlData;

    // hang while ros is ok
    while(ros::ok()) {
    }
 
    //std::cout << "finished" << std::endl;
    //delete quad;
    delete &pyb_listen_obj;
    delete desiredStateCommand;
    delete gaitScheduler;
    delete legController;
    delete stateEstimator;
    delete _tele;
    delete _controlData;
    delete fsm;

    return 0;
}
