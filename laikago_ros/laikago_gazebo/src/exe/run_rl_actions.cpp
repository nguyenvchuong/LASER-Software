/************************************************************************
Copyright (c) 2018-2019, Unitree Robotics.Co.Ltd. All rights reserved.
Use of this source code is governed by the MPL-2.0 license, see LICENSE.
************************************************************************/
/************************************************************************

This file: send state to rl, get actions from trained RL networks, apply to gazebo sim.

Keep broadcasting state, pyb should keep sending actions continuously


--------------------------------
receive data from pybullet, call MPC, send back to pybullet.
Service:
- MPCService, with PybulletFullState Request, and LegCmdArray Response
Subscribe:
- reset MPC (firstRun = true)
- set new Gait num

This is a service called from pybullet, no gazebo updates
- see usc_learning/ros_interface/run_mpc.py


************************************************************************/

#include "ros/ros.h"
//#include <gazebo_ros/gazebo_ros_api_plugin.h>
//#include <physics/physics.hh>
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
#include "laikago_msgs/RLService.h"
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/Imu.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/SetModelState.h>
#include <gazebo_msgs/SetLinkState.h>
#include <gazebo_msgs/GetJointProperties.h>
#include <gazebo_msgs/GetModelProperties.h>
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
#include "../../PybulletInterface/PybulletInterface.h"

// #include "gazebo/common/KeyFrame.hh"
// #include "gazebo/common/Animation.hh"
// #include "gazebo/common/Plugin.hh"
// #include "gazebo/common/Events.hh"
// #include "gazebo/common/Exception.hh"
// #include "gazebo/common/Console.hh"
// #include "gazebo/common/CommonTypes.hh"
// #include "gazebo/common/URI.hh"

// #include "gazebo/physics/Gripper.hh"
// #include "gazebo/physics/Joint.hh"
// #include "gazebo/physics/JointController.hh"
// #include "gazebo/physics/Link.hh"
// #include "gazebo/physics/World.hh"
// #include "gazebo/physics/PhysicsEngine.hh"
// #include "gazebo/physics/Model.hh"
// #include "gazebo/physics/Contact.hh"

// #include "gazebo/transport/Node.hh"

// #include "gazebo/util/IntrospectionManager.hh"
// #include "gazebo/util/OpenAL.hh"

// using namespace gazebo;
// using namespace physics;


using namespace std;
using namespace laikago_model;

class PybulletStateListener
{
public:
    PybulletStateListener() : Cmpc(0.001, 30),loop_rate(1000) {
        // don't need these for this file
        //pybulletState_sub = nm.subscribe("/laikago_gazebo/pybulletFullState", 1, &PybulletStateListener::PybStateCallback, this);
        //mpc_service = nm.advertiseService("/laikago_gazebo/mpc_comm", &PybulletStateListener::CallMPC, this); //persistent, true?
        //mpc_reset_sub = nm.subscribe("/laikago_gazebo/reset_mpc", 1, &PybulletStateListener::ResetMPC, this);
        //gait_sub = nm.subscribe("/laikago_gazebo/set_gait", 1, &PybulletStateListener::SetGait, this);
        //"/laikago_gazebo/rl_actions"
        rl_actions_sub = nm.subscribe("/laikago_gazebo/rl_actions", 1, &PybulletStateListener::SetLegCommands, this);
        

        gazebo_pause_srv = nm.serviceClient<std_srvs::Empty>("/gazebo/pause_physics"); 
        gazebo_unpause_srv = nm.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics"); 

        python_available_sub = nm.subscribe("/laikago_gazebo/python_available", 1, &PybulletStateListener::TurnOn, this);
        std::cout << "before client" << std::endl;
        rl_client = nm.serviceClient<laikago_msgs::RLService>("/laikago_gazebo/rl_comm");
        std::cout << "after client" << std::endl;
    }

    void TurnOn(const std_msgs::Empty& msg )
    {
        hasData = 1;
        std::cout << "turned on" << std::endl;
    }
    void GetRLCommands()
    {
        rl_srv.request.lowState = lowState;
        if(rl_client.call(rl_srv))
        {
            //unpause sim
            gazebo_unpause_srv.call(empty_srv);
            std::cout << "send leg cmds" << std::endl;

            // in a loop, update the leg cmds (pDes, kpCartesian, etc.) for 10ms (in RL chose actions every 10ms)


            SetLegCommands(rl_srv.response.legCmdArray);
            // spin, sim rate
            // pause sim
            loop_rate.sleep();
            std::cout << "sleep, unpause" << std::endl;
            gazebo_pause_srv.call(empty_srv);
        }
        else
        {
            std::cout << "invalid rl comm" << std::endl;
        }
    }
    /*
    Get leg commands from pybullet, set motor commands
    */
    void SetLegCommands(const laikago_msgs::LegCmdArray& msg)
    {
        for(int i = 0; i < 12; i++){
            lowCmd.motorCmd[i].torque = msg.motorCmd[i];
        }
        sendServoCmd();
        hasData = 1;
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
    ros::Subscriber python_available_sub; 
    ros::Subscriber pybulletState_sub, mpc_reset_sub, gait_sub; 
    ros::Subscriber rl_actions_sub;
    ros::ServiceServer mpc_service;
    ros::ServiceClient rl_client;
    laikago_msgs::LegCmdArray legCmdArray;
    laikago_msgs::RLService rl_srv;
    int gaitNum = 2;

    ros::ServiceClient gazebo_pause_srv, gazebo_unpause_srv;
    std_srvs::Empty empty_srv;
    ros::Rate loop_rate;
};


int main(int argc, char **argv)
{
    ros::init(argc, argv, "ros_mpc_interface");
    ros::Publisher lowCmd_pub, highCmd_pub; 
    ros::NodeHandle n;
    ros::Publisher lowState_pub; // full robot state
    ros::Publisher legCmds_pub, pybState_pub;  // desired leg commands from stand/MPC
    ros::ServiceClient reset_sim_srv; // reset sim at start of script
    ros::ServiceClient reset_world_srv; // reset world at start of script (difference with above?)
    ros::ServiceClient reset_joint_srv; 
    ros::ServiceClient gazebo_pause_srv, gazebo_unpause_srv;
    ros::ServiceClient set_model_state_serv;
    ros::ServiceClient get_link_state_srv, get_model_properties_srv, set_link_state_srv;
   
    ros::AsyncSpinner spinner(1); // one threads
    ros::Rate loop_rate(1000);
    spinner.start();

    // the following nodes have been initialized by "gazebo.launch"
    lowState_pub = n.advertise<laikago_msgs::LowState>("/laikago_gazebo/lowState/state", 1);
    legCmds_pub = n.advertise<laikago_msgs::LegCmdArray>("/laikago_gazebo/legCmds", 1);
    pybState_pub = n.advertise<laikago_msgs::PybulletState>("/laikago_gazebo/ros2pybstate", 1);
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

    // reset joints (just check if possible at first)
    // string joint_strings[12] ={"laikago_gazebo::FL_hip",
    //                                 "laikago_gazebo::FL_thigh",
    //                                 "laikago_gazebo::FL_calf",
    //                                 "laikago_gazebo::FR_hip",
    //                                 "laikago_gazebo::FR_thigh",
    //                                 "laikago_gazebo::FR_calf",
    //                                 "laikago_gazebo::RL_hip",
    //                                 "laikago_gazebo::RL_thigh",
    //                                 "laikago_gazebo::RL_calf",
    //                                 "laikago_gazebo::RR_hip",
    //                                 "laikago_gazebo::RR_thigh",
    //                                 "laikago_gazebo::RR_calf"};
    string link_names[12] ={"FL_hip",
                            "FL_thigh",
                            "FL_calf",
                            "FR_hip",
                            "FR_thigh",
                            "FR_calf",
                            "RL_hip",
                            "RL_thigh",
                            "RL_calf",
                            "RR_hip",
                            "RR_thigh",
                            "RR_calf"};
    string joint_strings[12] ={
                            "FL_hip_joint",
                            "FL_thigh_joint",
                            "FL_calf_joint",
                            "FR_hip_joint",
                            "FR_thigh_joint",
                            "FR_calf_joint",
                            "RL_hip_joint",
                            "RL_thigh_joint",
                            "RL_calf_joint",
                            "RR_hip_joint",
                            "RR_thigh_joint",
                            "RR_calf_joint"};



    // add simulation reset, so we don't have to keep relaunching ROS if robot falls over
    // pause/unpause
    gazebo_pause_srv = n.serviceClient<std_srvs::Empty>("/gazebo/pause_physics"); 
    gazebo_unpause_srv = n.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics"); 
    //reset sim/world
    reset_sim_srv = n.serviceClient<std_srvs::Empty>("/gazebo/reset_simulation"); 
    reset_world_srv = n.serviceClient<std_srvs::Empty>("/gazebo/reset_world");
    set_model_state_serv = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
    //reset joint positions 
    reset_joint_srv = n.serviceClient<gazebo_msgs::SetModelConfiguration>("/gazebo/set_model_configuration"); 
    // reset world/sim (what's the difference? does not reset joints?)
    std_srvs::Empty empty_srv;
    // get link states
    get_link_state_srv = n.serviceClient<gazebo_msgs::GetJointProperties>("/gazebo/get_joint_properties");
    get_model_properties_srv = n.serviceClient<gazebo_msgs::GetModelProperties>("/gazebo/get_model_properties");
    set_link_state_srv = n.serviceClient<gazebo_msgs::SetLinkState>("/gazebo/set_link_state");


    gazebo_msgs::SetLinkState lnk;

    motion_reset();

    // set everything exactly
    // for (int i = 0; i<12; i++){
    //     lnk.request.link_state.link_name = link_names[i];
    //     lnk.request.link_state.pose.position.x = 0;
    //     lnk.request.link_state.pose.position.y = 0;
    //     lnk.request.link_state.pose.position.z = 0;


    //     set_link_state_srv.call(lnk);
    //     std::cout << "status " << lnk.response.status_message << std::endl;
    //     std::cout << "status " << lnk.response.success << std::endl;
    //     //clearBodyWrenches(link_names[i]);
    //     //clearJointForces(link_names[i]);
    // }
    // clearBodyWrenches();
    // clearJointForces();
    //ResetPhysicsStates();

    // before any init
    // for (int i=0; i<12; i++){
    //     gazebo_msgs::GetJointProperties jnt;
    //     jnt.request.joint_name = joint_strings[i];
    //     bool output = get_link_state_srv.call(jnt);
    //     if (output) {
    //         std::cout << "joint " << joint_strings[i] << std::endl;
    //         std::cout << "status " << jnt.response.status_message << std::endl;
    //         std::cout << "pos " << jnt.response.position[0] << " rate " << jnt.response.rate[0] << std::endl;
    //     }
        
    // }

    // reset joints (why does this not work?)
    // motion_init();
    // sendServoCmd();

    usleep(1000000);
    // std::cout << "pause" << std::endl;
    // //gazebo_pause_srv.call(empty_srv);
    // usleep(1000000);
    // std::cout << "reset world" << std::endl;
    // reset_world_srv.call(empty_srv); // just puts base at origin
    // usleep(1000000);
    // std::cout << "reset sim" << std::endl;
    // reset_sim_srv.call(empty_srv); // also sets joints to 0

    // gazebo_msgs::SetModelConfiguration model_config_srv;

    // model_config_srv.request.model_name = "laikago_gazebo";
    // model_config_srv.request.urdf_param_name = "robot_description";


    // double joint_pos[12] = {0.0, 1.22, -2.77, 
    //                         0.0, 1.22, -2.77, 
    //                         0.0, 1.22, -2.77, 
    //                         0.0, 1.22, -2.77};
    // for(int i = 0; i < 12; i++){
    //     model_config_srv.request.joint_names.push_back(joint_strings[i]);
    //     model_config_srv.request.joint_positions.push_back(joint_pos[i]);
    // }
    // usleep(1000000);
    std::cout << "set joints" << std::endl;
    //reset_joint_srv.call(model_config_srv);
    //std::cout << reset_joint_srv.call(model_config_srv) << std::endl;


    gazebo_msgs::SetModelState model_state;
    model_state.request.model_state.model_name = "laikago_gazebo";
    model_state.request.model_state.pose.position.z = 0.5;
    //model_state.request.model_state.pose.position.x = -5;
    std::cout << "set model" << std::endl;
    std::cout << set_model_state_serv.call(model_state) << std::endl;
    usleep(1000000);

    // for(int i = 0; i < 12; i++){
    //     lowCmd.motorCmd[i].torque = 0;
    // }
    // motion_init();
    // sendServoCmd();
    
    // gazebo_pause_srv.call(empty_srv);
    // usleep(1000000);
    // gazebo_unpause_srv.call(empty_srv);


    // check model
    // gazebo_msgs::GetModelProperties mdl;
    // mdl.request.model_name = "laikago_gazebo";
    // bool output = get_model_properties_srv.call(mdl);
    // if (output) {
    //     std::cout << "status " << mdl.response.status_message << std::endl;
    //     std::cout << "is_static " << mdl.response.is_static << std::endl;
    //     std::cout << "is_static " << mdl.response.is_static << std::endl;
    //     for (int i=0; i<12; i++){
    //         std::cout << "------------------------------------------" << std::endl;
    //         std::cout << "body_names " << mdl.response.body_names[i] << std::endl;
    //         std::cout << "geom_names " << mdl.response.geom_names[i] << std::endl;
    //         std::cout << "joint_names " << mdl.response.joint_names[i] << std::endl;
    //     }
        
    // }

    // before any init
    // for (int i=0; i<12; i++){
    //     gazebo_msgs::GetJointProperties jnt;
    //     jnt.request.joint_name = joint_strings[i];
    //     bool output = get_link_state_srv.call(jnt);
    //     if (output) {
    //         std::cout << "joint " << joint_strings[i] << std::endl;
    //         std::cout << "status " << jnt.response.status_message << std::endl;
    //         std::cout << "pos " << jnt.response.position[0] << " rate " << jnt.response.rate[0] << std::endl;
    //     }
        
    // }

    // lowState must be updated with pybullet data
    CurrentState listen_publish_obj;

    usleep(300000); // must wait 300ms, to get first state
   
    //laikago_msgs::LegCmdArray legCmdArray;
    laikago_msgs::PybulletFullState ros2pybstate;

    
    std::cout << "stand up" << std::endl;
    double dt = 0.001;
    Quadruped quad ;
    quad.setQuadruped(2); // 1 for Aliengo, 2 for A1
    // initialize new leg controller and state estimate object
    StateEstimate stateEstimate;
    std::cout << "start Leg Controller" << std::endl;
    LegController* legController = new LegController(quad);

    // zero out
    legController->zeroCommand();
    sendServoCmd();

    std::cout << "start state estimate" << std::endl;
    StateEstimatorContainer* stateEstimator = new StateEstimatorContainer(
                       &lowState.imu, legController->data,&stateEstimate);
    // cheat data (ground truth)
    // stateEstimator->addEstimator<ContactEstimator>();
    // stateEstimator->addEstimator<CheaterOrientationEstimator>();
    // stateEstimator->addEstimator<CheaterPositionVelocityEstimator>();
    // using sensors 
    stateEstimator->addEstimator<ContactEstimator>();
    stateEstimator->addEstimator<VectorNavOrientationEstimator>();
    stateEstimator->addEstimator<TunedKFPositionVelocityEstimator>();

    // Vec4<double> contactphase(0.5,0.5,0.5,0.5); 
    // stateEstimator->setContactPhase(contactphase);


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
    motion_init();
    
    // Start interface obj
    //PybulletStateListener pyb_listen_obj; 
    //pyb_listen_obj._controlData = _controlData;
    PybulletInterface pyb_listen_obj;
    pyb_listen_obj._controlData = _controlData;

    // extraneous
    legController->updateData();
    stateEstimator->run();

    int counter = 0;

    // record velocity 
    // write out trajectory
    std::ofstream vel_traj("velocities.txt");
    std::vector<double> vel_state(12);
    pyb_listen_obj.getVelocities(vel_state);
    for(int j=0; j<12; j++)
        vel_traj << vel_state[j] << " ";
    vel_traj << "\n";

    // record full state 
    // write out trajectory
    int full_state_len = 76;
    std::ofstream full_traj("full_state.txt");
    std::vector<double> full_state(full_state_len);
    pyb_listen_obj.getFullState(full_state);
    for(int j=0; j<full_state_len; j++)
        full_traj << full_state[j] << " ";
    full_traj << "\n";



    /****************************************************************************
    Have robot stand up first with balance controller 
    /****************************************************************************/
    // while(ros::ok() && !pyb_listen_obj.hasData && counter < 2000) {
    //     // update current robot state
    //     _controlData->_legController->updateData();
    //     _controlData->_stateEstimator->run();

    //     //pyb_listen_obj._controlData = _controlData; // shared pointers, updates both
    //     // use pybullet interface
    //     pyb_listen_obj.runBalanceController();

    //     // update control data, this should always be called in the loop to compute fresh torques 
    //     // from desired pDes, vDes, ffwd, etc.
    //     _controlData->_legController->updateCommand();

    //     // publish robot state
    //     lowState_pub.publish(lowState);
    //     pyb_listen_obj.sendObservation();
    //     loop_rate.sleep();

    //     counter++;
    // }
    motion_reset();
    paramInit();
    counter = 0;

    double des_run_vel = 2;
    double run_vel_time = 3;
    /****************************************************************************
    [ALL IN C] query robot state and run policy network all in C++
    /****************************************************************************/
    while(ros::ok() && counter<20000) {

        // update current robot state
        _controlData->_legController->updateData();
        _controlData->_stateEstimator->run();

        double sim_t = counter*0.001;
        if(sim_t <8)
            pyb_listen_obj.setSimTimeObs(10 - counter*0.001);
        else
            pyb_listen_obj.setSimTimeObs(2);

        if (pyb_listen_obj.useDesVel()){
            if (sim_t < run_vel_time){
                double vel = des_run_vel * (sim_t) / (run_vel_time);
                pyb_listen_obj.setDesVelObs(vel);
            }
            else{
                pyb_listen_obj.setDesVelObs(des_run_vel);
            }

        }

        // call and set legCommands from policy network
        // can call every time step or every 10, both work - but less frequent is MORE jittery for balance
        if (counter % 10 == 0){ //to match pybullet - choose new high level command every 0.01 s 
            pyb_listen_obj.computeAction();
        }


        // write out velocities
        pyb_listen_obj.getVelocities(vel_state);
        for(int j=0; j<12; j++)
            vel_traj << vel_state[j] << " ";
        vel_traj << "\n";
        // write out full state 
        pyb_listen_obj.getFullState(full_state);
        for(int j=0; j<full_state_len; j++)
            full_traj << full_state[j] << " ";
        full_traj << "\n";

        // update control data, this should always be called in the loop to compute fresh torques 
        // from desired pDes, vDes, ffwd, etc.
        // _controlData->_legController->updateCommand();
        pyb_listen_obj.sendCommand();
        loop_rate.sleep();

        counter++;

    }
    vel_traj.close();
    full_traj.close();
    /****************************************************************************
    [FROM PYTHON] Send robot state in a loop, receiving action as well.
    /****************************************************************************/

    //pause 
    //gazebo_pause_srv.call(empty_srv);
    /*
    while(ros::ok()) {

        // update current robot state
        _controlData->_legController->updateData();
        _controlData->_stateEstimator->run();

        // pybullet interface obj should be updating the leg commands, hopefully at least at 100 Hz
        //pyb_listen_obj.runBalanceController();


        // update control data, this should always be called in the loop to compute fresh torques 
        // from desired pDes, vDes, ffwd, etc.
        _controlData->_legController->updateCommand();
        // publish robot state
        lowState_pub.publish(lowState);
        pyb_listen_obj.sendObservation();
        loop_rate.sleep();


        //pyb_listen_obj.getRLCommandsService();

        // set motorCmd from pybullet
        // update leg cmd
        // sendServoCmd();

        //lowState_pub.publish(lowState);
        //loop_rate.sleep();
        //std::cout << "looping" << std::endl;
    }
    */
    motion_init();
    // sendServoCmd();
    std::cout << "finished" << std::endl;
    //delete quad;
    //delete &pyb_listen_obj;
    delete desiredStateCommand;
    delete gaitScheduler;
    delete legController;
    delete stateEstimator;
    delete _tele;
    delete _controlData;
    delete fsm;

    return 0;
}
