/************************************************************************

This is the executable for running the MPC while actions are chosen with RL. 

This file: receive RL foot positions / desired body velocity, call MPC.

Subscribe:
- set MPC foot positions
- reset MPC (firstRun = true)
- set new Gait num

Service: 
- reset gazebo sim

Python Gym env:
- see usc_learning/ros_only/ros_gym.py


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
#include "laikago_msgs/RLFootPos.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
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
#include "../../BalanceController/BalanceController.hpp"
#include "../../PybulletInterface/PybulletInterface.h"

using namespace std;
using namespace laikago_model;

class RLGymHelper
{
public:
    RLGymHelper() : Cmpc(0.001, 30), v_des(0, 0, 0), loop_rate(1000) {
        //mpc_service = nm.advertiseService("/laikago_gazebo/mpc_comm", &PybulletStateListener::CallMPC, this); //persistent, true?
        mpc_reset_sub = nm.subscribe("/laikago_gazebo/reset_mpc", 1, &RLGymHelper::ResetMPC, this);
        gait_sub = nm.subscribe("/laikago_gazebo/set_gait", 1, &RLGymHelper::SetGait, this);
        rl_foot_sub = nm.subscribe("/laikago_gazebo/set_rl_foot_pos", 1, &RLGymHelper::SetRLFootPositions, this);

        env_reset_sub = nm.subscribe("/laikago_gazebo/reset_env", 1, &RLGymHelper::ResetEnv, this);
        env_reset_service = nm.advertiseService("/laikago_gazebo/reset_env_service", &RLGymHelper::ResetEnvService, this); //persistent, true?

        //gazebo specific, for reset
        reset_sim_srv = nm.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation"); 
        reset_world_srv = nm.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
        set_model_state_serv = nm.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");

    }
    void SetRLFootPositions(const laikago_msgs::RLFootPos& msg){        
        //double rlFootPos[8];
        for(int i = 0; i<8; i++){
            rlFootPos[i] = msg.rlFootPos[i];
        }
        //Cmpc.setRLFootPositions(msg.pybState.rlFootPos);
        Cmpc.setRLFootPositions(rlFootPos);
        if (msg.velFlag){
            v_des << msg.v_des[0], msg.v_des[1], msg.v_des[2];
        }
        else{
            // just go slightly faster
            v_des = _controlData->_stateEstimator->getResult().vBody;
            v_des[0] += 0.01;
            // max vel
            v_des[0] = max ( min( (double) v_des[0], (double) 1), (double) 0 );
            v_des[1] = 0;
            v_des[2] = 0; 
        }
    }

    /*
     * Reset whole simulation, standup, etc. 
     */ 
    void ResetEnv(){
        resetting_flag = true;
        v_des << 0, 0, 0;
        motion_init();
        motion_reset();
        std::cout << "MOVE TO CROUCH" << std::endl;
        //motion_reset_ground();

        // reset to "crouch" init instead
        // double pos_laydown[12] = {  0.0, 0.5, -1., -0.0, 0.5, -1., 
        //                             0.0, 0.5, -1., -0.0, 0.5, -1.};
        // moveAllPosition(pos_laydown, 5000);

        //motion_init();
        //paramInit();
        //std::cout << "reset world" << std::endl;
        //reset_world_srv.call(empty_srv); // just puts base at origin
        //usleep(1000000);
        std::cout << "reset sim" << std::endl;
        reset_sim_srv.call(empty_srv); // also sets joints to 0
        //usleep(1000000);
        std::cout << "motion init" << std::endl;
        motion_init();
        //motion_reset_ground();
        //paramInit();
        sendServoCmd();
        for (int i=0; i < 2000; i++){
            _controlData->_legController->updateData();
            _controlData->_stateEstimator->run();
            sendServoCmd();
            loop_rate.sleep();
        }
        //motion_init();


        // add gazebo reset
        // gazebo_msgs::SetModelState model_state;
        // model_state.request.model_state.model_name = "laikago_gazebo";
        // model_state.request.model_state.pose.position.z = 0.4;
        // std::cout << "set model" << std::endl;
        // std::cout << set_model_state_serv.call(model_state) << std::endl;
        // usleep(10000);

        //usleep(1000000);
        _controlData->_legController->updateData();
        _controlData->_stateEstimator->run();
        // stand up
        //StandUpImpedance();
        RunBalanceController();
        //usleep(10000);
        resetting_flag = false;
    }
    void ResetEnv(const std_msgs::Empty& msg)
    {
        ResetEnv();
    }
    bool ResetEnvService(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
    {
        ResetMPC(empty_msg);
        ResetEnv();
        return true;
    }
 

    void ResetMPC(const std_msgs::Empty& msg)
    {
        Cmpc.reset(); // verify this is enough to reset
        //Cmpc.useRLPos = false; // already in function
        std::cout << "reset MPC" << std::endl;
    }
    void SetGait(const std_msgs::Int32& msg)
    {
        gaitNum = msg.data;
        Cmpc.setGaitNum(gaitNum);
    }

    void StandUpImpedance(){
        counter = 0;
      
        // initial foot position
        Mat34<double> init_foot_pos;
        double side_sign[4] = {-1, 1, -1, 1};
        _controlData->_legController->updateData();
        // for(int i = 0; i < 4; i++){
        //     init_foot_pos.col(i) = _controlData->_legController->data[i].p;
        // }
        for(int i = 0; i < 4; i++){
            init_foot_pos.col(i) << 0, side_sign[i] * 0.083, 0;
        }
        double h = 0.3; // standup height
        //double side_sign[4] = {-1, 1, -1, 1};
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
                // std::cout << "leg  " << i << "  " << _controlData->_legController->data[i].p << std::endl;
            }

            _controlData->_legController->updateCommand();
            counter++;
            //lowState_pub.publish(lowState);

            loop_rate.sleep();
        }

    }

    void RunBalanceController(){
        // reset forces and steps to 0

        counter = 0;
        _controlData->_legController->updateData();
        _controlData->_stateEstimator->run();
        // se_xfb[3] = 1.0; 
        //  int during = 10;
        // position & rpy desired
        for(int i = 0; i < 3; i++){
            p_des[i] = _controlData->_stateEstimator->getResult().position(i); 
            v_des_b[i] = 0;
            omegaDes[i] = 0;
            rpy[i] = 0;
        }
        rpy[2] = _controlData->_stateEstimator->getResult().rpy(2);
        p_des[2] = 0.3;

        while(ros::ok() && counter < 1000) {
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
            balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des_b);
            balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
            balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des_b, v_act,
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
            counter += 1;
        
        }
        counter = 0;
    }

    int hasData = 0;
    Vec3<double> v_des;
    double yaw_rate;
    StateEstimate result;
    ControlFSMData* _controlData;
    ConvexMPCLocomotion Cmpc;
    BalanceController balanceController;

    double rlFootPos[8];
    bool resetting_flag = true;
    int counter = 0;


private:
    ros::NodeHandle nm;
    //ros::Subscriber pybulletState_sub; 
    ros::Subscriber mpc_reset_sub, gait_sub, rl_foot_sub, env_reset_sub;
    ros::ServiceServer env_reset_service ;//mpc_service;
    //laikago_msgs::LegCmdArray legCmdArray;
    ros::Rate loop_rate;

    int gaitNum = 2;

    // gazebo, for reset
    ros::ServiceClient reset_sim_srv; // reset sim at start of script
    ros::ServiceClient reset_world_srv; // reset world at start of script (difference with above?)
    ros::ServiceClient set_model_state_serv;

    std_srvs::Empty empty_srv;
    std_msgs::Empty empty_msg;

    // ===================== Balance Controller
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
    double pFeet[12], p_act[3], v_act[3], O_err[3], rpy[3], p_des[3], v_des_b[3], omegaDes[3]; //v_des[3]
    double se_xfb[13];
    double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];
    double b_control[6];
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "mpc_rl_foot_pos");
    ros::Publisher lowCmd_pub, highCmd_pub; 
    ros::NodeHandle n;
    ros::Publisher lowState_pub; // full robot state
    ros::Publisher legCmds_pub, pybState_pub;  // desired leg commands from stand/MPC
   
    ros::AsyncSpinner spinner(1); // one threads
    ros::Rate loop_rate(1000);
    spinner.start();

    // must create this object to update lowState with model state
    CurrentState listen_publish_obj;

    usleep(300000); // must wait 300ms, to get first state
   
    //laikago_msgs::LegCmdArray legCmdArray;
    //laikago_msgs::PybulletState ros2pybstate;

    // the following nodes have been initialized by "gazebo.launch"
    lowState_pub = n.advertise<laikago_msgs::LowState>("/laikago_gazebo/lowState/state", 1);
    legCmds_pub = n.advertise<laikago_msgs::LegCmdArray>("/laikago_gazebo/legCmds", 1);
    //pybState_pub = n.advertise<laikago_msgs::PybulletState>("/laikago_gazebo/ros2pybstate", 1);
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

    std::cout << "init" << std::endl;
    // FSM_State* fsm = new FSM_State(_controlData);
    // std::cout << "FSM_Started" << std::endl;
    motion_init();

    legController->updateData();
    stateEstimator->run();
    //stand();
    
    // Start interface obj
    //PybulletStateListener pyb_listen_obj; 
    //pyb_listen_obj._controlData = _controlData;
    //PybulletInterface pyb_listen_obj;
    //pyb_listen_obj._controlData = _controlData;

    RLGymHelper rl_gym_obj;
    rl_gym_obj._controlData = _controlData;

    /****************************************************************************

    This is the fsm->PDStand function above, but with the lowState publish state.
    Run at first to stand, then MPC

    /****************************************************************************/
    // reset environment
    rl_gym_obj.ResetEnv();
    // standup
    //rl_gym_obj.StandUpImpedance();
    int counter = 0;
  
    // // initial foot position
    // Mat34<double> init_foot_pos;
    // _controlData->_legController->updateData();
    // for(int i = 0; i < 4; i++){
    //     init_foot_pos.col(i) = _controlData->_legController->data[i].p;
    // }
    // double h = 0.3; // standup height
    // double side_sign[4] = {-1, 1, -1, 1};
    // Vec3<double> ff_force_world(0, 0, 0);

    // while(ros::ok() && counter < 1000){
    //     double progress = counter * 0.001;
    //     if(progress > 1.)  {progress = 1.;}

    //     _controlData->_legController->updateData();  
    //     _controlData->_stateEstimator->run();    

    //     for(int i = 0; i < 4; i++){
    //         _controlData->_legController->commands[i].kpCartesian = Vec3<double>(400,400,900).asDiagonal();
    //         _controlData->_legController->commands[i].kdCartesian = Vec3<double>(20,20,20).asDiagonal();

    //         _controlData->_legController->commands[i].pDes << 0, side_sign[i] * 0.083, 0;

    //         _controlData->_legController->commands[i].pDes[2] = -h*progress + (1. - progress) * init_foot_pos(2, i);
    //         // _data->_legController->commands[i].feedforwardForce = _data->_stateEstimator->getResult().rBody.transpose() * ff_force_world;
    //         // std::cout << "leg  " << i << "  " << _data->_legController->data[i].p << std::endl;
    //     }

    //     _controlData->_legController->updateCommand();
    //     counter++;
    //     //lowState_pub.publish(lowState);

    //     loop_rate.sleep();
    // }

    /*======================== Test standalone MPC  ====================*/
    /*
    // define CMPC
    ConvexMPCLocomotion Cmpc(0.001, 30);
    counter = 0;
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

        // run MPC, only at MPC frequency 
        Cmpc.run(*_controlData);
        // send control to robot
        _controlData->_legController->updateCommand();

        loop_rate.sleep();
        //mpc_loop_rate.sleep();
        counter++;
    }
    */
    /****************************************************************************

    Now standing, so run MPC with RL foot positions. 

    /****************************************************************************/


    // define CMPC
    //ConvexMPCLocomotion Cmpc(0.001, 30);
    counter = 0;
    int call_mpc_frequency = 1; //1 is call every time 
    double yaw_rate = 0;
    /*======================== MPC locomotion ====================*/
    
    while(ros::ok() ){ //&& counter < 1000
         //if (!rl_gym_obj.resetting_flag){
        _controlData->_legController->updateData();
        _controlData->_stateEstimator->run();
        //Vec3<double> v_des(0, 0, 0);
        yaw_rate = 0;
        //Cmpc.setGaitNum(2);
        if(fabs(_controlData->_stateEstimator->getResult().rpy[2]) > 0.01){
            if(_controlData->_stateEstimator->getResult().rpy[2] < 0)
                yaw_rate = 0.5;
            else yaw_rate = -0.5;
            //_controlData->_desiredStateCommand->setStateCommands(v_des, yaw_rate);
        }

        if (rl_gym_obj.resetting_flag){
            yaw_rate = 0;
        }

        //_controlData->_desiredStateCommand->setStateCommands(v_des, yaw_rate);
        _controlData->_desiredStateCommand->setStateCommands(rl_gym_obj.v_des, yaw_rate);
        // run MPC
        rl_gym_obj.Cmpc.run(*_controlData);

        // if not resetting, send control to robot
        if (!rl_gym_obj.resetting_flag){
            _controlData->_legController->updateCommand();
        }
               
       // _controlData->_legController->updateCommand();
        

        loop_rate.sleep();
        //mpc_loop_rate.sleep();
        counter++; // this will overflow 

        // do this for now, afterwards need to use StateEstimate (write interface in python/rospy)
        // pyb_listen_obj.sendObservation();
        // loop_rate.sleep();
        //}
    }
    
 
    //std::cout << "finished" << std::endl;
    //delete quad;
    //delete &pyb_listen_obj;
    delete desiredStateCommand;
    delete gaitScheduler;
    delete legController;
    delete stateEstimator;
    delete _tele;
    delete _controlData;
    // delete fsm;

    return 0;
}
