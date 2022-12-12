/************************************************************************

This is the executable for interacting between ROS and pybullet.
See PybulletInterface for details.

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
// #include "laikago_msgs/HighCmd.h"
// #include "laikago_msgs/HighState.h"
// #include "laikago_msgs/LowCmd.h"
// #include "laikago_msgs/LowState.h"
// #include "laikago_msgs/MotorCmd.h"
// #include "laikago_msgs/MotorState.h"
// #include "laikago_msgs/LegCmdArray.h"
// #include "laikago_msgs/LegCmd.h"
// #include "laikago_msgs/PybulletState.h"
// #include "laikago_msgs/PybulletFullState.h"
// #include "laikago_msgs/MPCService.h"
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
#include "../../PybulletInterface/PybulletInterface.h"

#include "../../cppflow/include/Model.h"
#include "../../cppflow/include/Tensor.h"
#include <numeric>
#include <iomanip>

using namespace std;
using namespace laikago_model;


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
    quad.setQuadruped(2); // 1 for Aliengo, 2 for A1
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
    //PybulletStateListener pyb_listen_obj; 
    //pyb_listen_obj._controlData = _controlData;
    // PybulletInterface pyb_listen_obj;
    // pyb_listen_obj._controlData = _controlData;

    // hang while ros is ok
    // while(ros::ok()) {
    // }

    // test tf
    // Model model("/home/guillaume/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/"
    //              "usc_learning/learning/rllib/temp11/model.pb"); //temp5/model/saved_model.pb");
    // Model model("/home/guillaume/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/"
    //              "usc_learning/learning/rllib/exported/graph.pb"); //temp5/model/saved_model.pb");
    // //Model model("/home/guillaume/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/usc_learning/learning/rllib/temp5/checkpoint/model.meta");
    // std::cout << 'a' << std::endl;
    // model.restore("/home/guillaume/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/"
    //                 "usc_learning/learning/rllib/exported/my_model");
    // //model.init();

    // EPFL
    Model model("/home/bellegar/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/"
                 "usc_learning/learning/rllib/model/graph.pb"); //temp5/model/saved_model.pb");
    std::cout << 'a' << std::endl;
    model.restore("/home/bellegar/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/"
                    "usc_learning/learning/rllib/model/my_model");


    std::vector<std::string> result = model.get_operations();

    // Tensor observation{model, "default_policy/observation"};
    // Tensor action{model, "default_policy/cond/Merge"};
    // Tensor action_1{model, "default_policy/cond_1/Merge"};

    // EPFL
    Tensor observation{model, "default_policy/obs"};
    Tensor action{model, "default_policy/cond/Merge"};
    Tensor action_1{model, "default_policy/cond_1/Merge"};

    // std::vector<float> data(75); //was 48
    std::vector<float> data(65); //was 48
    std::iota(data.begin(), data.end(), 0);

    std::cout << "set data " << std::endl;
    observation.set_data(data);

    std::cout << "run model"  << std::endl;
    model.run({&observation}, {&action, &action_1});
    for (float f : action.get_data<float>()) {
        std::cout << f << " ";
    }
    for (float f : action_1.get_data<float>()) {
        std::cout << f << " ";
    }
    std::cout << std::endl;
 
    //std::cout << "finished" << std::endl;
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
