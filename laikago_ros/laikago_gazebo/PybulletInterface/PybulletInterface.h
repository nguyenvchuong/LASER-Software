/************************************************************************

PybulletInterface, includes 
-communication 
-interface to MPC, BalanceController
-options for setting simulation state
-get actions from RL and set controls

-eventually may use as a basis for a gym environment class

This file: send state to rl, get actions from trained RL networks, apply to gazebo sim.

Keep broadcasting state, pyb should keep sending actions continuously


--------------------------------
receive data from pybullet, call MPC, send back to pybullet.
Service:
- MPCService, with PybulletFullState Request, and LegCmdArray Response
Subscribe:
- reset MPC (firstRun = true)
- set new Gait num

For pybullet interface see
-  usc_learning/ros_interface/run_mpc.py


************************************************************************/
#ifndef PYBULLETINTERFACE_H
#define PYBULLETINTERFACE_H

#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include "laikago_msgs/PybulletFullState.h"
#include "laikago_msgs/MPCService.h"
#include "laikago_msgs/RLService.h"
#include "laikago_msgs/RLObs.h"
#include "laikago_msgs/RLAct.h"
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <std_srvs/Empty.h>
#include <gazebo_msgs/SetModelConfiguration.h>
#include <gazebo_msgs/SetModelState.h>
#include <vector>
#include <string.h>
#include <math.h>
#include "../ConvexMPC/ConvexMPCLocomotion.h"
#include "../BalanceController/BalanceController.hpp"
#include "../include/Utilities/Timer.h"

// #include "../CppFlow/Model.h"
// #include "../CppFlow/Tensor.h"
// #include "../cppflow/include/Model.h"
// #include "../cppflow/include/Tensor.h"
#include "../cppflow/include/cppflow/model.h"
#include "../cppflow/include/cppflow/tensor.h"
#include "../cppflow/include/cppflow/cppflow.h"
#include <numeric>
#include <iomanip>

#include "laikago_msgs/TFService.h"
//using namespace std;
//using namespace laikago_model;

class PybulletInterface
{
public:
    // TODO: add initialization for selecting mode - i.e. for MPC only, balance controller, RL service, etc.
    PybulletInterface(); 

    /**
     * Flag callback that indicates we should start listening for (other) data.
     */
    void turnOn(const std_msgs::Empty& msg );

    /**
     * Get RL commands, set in ROS, used as service 
     */
    void getRLCommandsService();

    /**
     * Get leg commands from pybullet, set motor commands
     *
     */
    void setLegCommands(const laikago_msgs::LegCmdArray& msg);

    /**
     * Sets the full robot state from pybullet, and also v_des and des yaw_rate for MPC.
     */
    void setStateFromPybullet(const laikago_msgs::PybulletFullState& pybState);

    /**
     * Set LegCmdArray from current (or computed) ROS state, usually to be sent to pybullet
     */
    void setResponseLegCmdArray(laikago_msgs::LegCmdArray& legCmdArray);

    /************************ Balance controller *************************/
    /**
     * Call Balance Controller from pybullet as a service
     */
    bool callBalanceController(laikago_msgs::MPCServiceRequest& msg, laikago_msgs::MPCServiceResponse& response);
    /**
     * Actual computation for balance controller
     */
    void runBalanceController();
    /**
     * Reset, so next call to runBalanceController will try to keep base at new x,y position
     *  -without this, body drifts and then falls down
     */
    void resetBalanceController(const std_msgs::Empty& msg);

    /**
     * TODO: add function to set desired p_des for balance controller,  
     *  -for example, may want to be at center of current feet position
     */

    /****************************** MPC **********************************/
    /**
     * Service callback: get state from Pybullet, call MPC and return leg commands
     */
    bool callMPC(laikago_msgs::MPCServiceRequest& msg, laikago_msgs::MPCServiceResponse& response);
    /**
     * Reset MPC: just sets firstRun=true (for now)
     */
    void resetMPC(const std_msgs::Empty& msg);
    /**
     * Set gait, 1-6, see ConvexMPC
     */
    void setGait(const std_msgs::Int32& msg);
    //void setMPCFootPos(const std_msgs::32& msg);


    /************************** RL obs/act ******************************/
    /**
     * Preprocess lowState/StateEstimate to observation format for RL, following default_contacts format
     */
    void getObservation();
    /**
     * No cheater state observation 
     */
    void getRealObservation();
    /**
     * No z
     */
    void getMinimalObservation();
    /**
     * Set desired velocity  in obs
     */
    void setDesVelObs(double vel);
    /**
     * Set simulation time remaining in obs
     */
    void setSimTimeObs(double t);
    /**
    *  Place velocities in vector 
    */
    void getVelocities(std::vector<double>& vel);
    /**
     * Place full state in vector
     * Body: pos xyz
             rpy
             lin vel
             ang vel
        joint pos
            vel
            torques
        foot pos
             vel
    */
    void getFullState(std::vector<double>& full_state);
    /**
     * getObservation, and publish
     */
    void sendObservation();
    /**
     * Process observation like in VecNormalize, need mean and std dev (read in from files)
     */
    void normalizeObservation(std::vector<float>& obs);
    /**
     * full RL method: get obs, scale, send to NN, apply leg commands; named like rllib
    */
    void computeAction();
    /**
     * GENERAL set leg commands from RL (for native C++)
     */
    void generalSetLegCommandsRL();
    /**
     * Set leg commands from RL
     */
    void processAndSetLegCommandsFromRL(const laikago_msgs::RLAct& msg);

    // actually send command
    void sendCommand();

    /************************** Tensorflow  ******************************/
    /**
     * Setup tensorflow graphs and placeholders.
     */
    void setupTensorflow();
    /**
     * Query tensorflow service as test from pybullet.
     */
    bool queryTensorflowModelService(laikago_msgs::TFServiceRequest& msg, laikago_msgs::TFServiceResponse& response);

    /**
     * Load obs and action spaces
     */
    bool loadRLSpaceParams();

    bool useDesVel(){return des_vel_in_obs;};
    int hasData = 0;
    Vec3<double> v_des_mpc;
    double yaw_rate;
    StateEstimate result;
    ControlFSMData* _controlData;
    ConvexMPCLocomotion Cmpc;
    double rlFootPos[8];

    // Balance controller 
    BalanceController balanceController;


private:
    ros::NodeHandle nm;
    // subscribers
    ros::Subscriber python_available_sub; 
    ros::Subscriber pybulletState_sub, mpc_reset_sub, mpc_gait_sub, balance_reset_sub, mpc_set_rl_foot_pos_sub; 
    ros::Subscriber rl_actions_sub;
    // services (servers/clients)
    ros::ServiceServer mpc_service, balance_service;
    ros::ServiceClient rl_client;
    // msgs
    laikago_msgs::LegCmdArray legCmdArray;
    laikago_msgs::RLService rl_srv;
    int gaitNum = 2;

    // gazebo sim 
    ros::ServiceClient gazebo_pause_srv, gazebo_unpause_srv;
    std_srvs::Empty empty_srv;
    ros::Rate loop_rate;

    //Timer mytime;
    /************************** RL post/preprocess *************************/
    ros::Publisher  rl_obs_pub;
    ros::Subscriber rl_act_sub;
    laikago_msgs::RLObs rl_obs;
    laikago_msgs::RLAct rl_act;

    /************************** Tensorflow utils *************************/
    // std::string tf_root = "src/USC-AlienGo-Software/laikago_ros/rl_data/orig_balance/";
    // std::string tf_root = "/home/guillaume/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/"
    //                         "usc_learning/learning/rllib/exported_model_des_vel/";

    // this is the one for all the paper videos and data
    // std::string tf_root = "/home/guillaume/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/"
                            // "usc_learning/learning/rllib/exported_model_feb24_paper/";
    // std::string tf_root = "src/USC-AlienGo-Software/laikago_ros/rl_data/exported_model_feb24_paper/";
    // std::string tf_root = "src/LASER-Software/laikago_ros/rl_data/exported_model_feb24_paper/";

    // test other ideal condition networks
    // std::string tf_root = "/home/guillaume/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/"
    //                         "usc_learning/learning/rllib/model_mu_02_dynrand_v1/";
    // std::string tf_root = "/home/guillaume/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/"
    //                         "usc_learning/learning/rllib/model_pd/";

    // new tests at EPFL
    // std::string tf_root = "/home/bellegar/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/"
                            // "usc_learning/learning/rllib/model/";
    // std::string tf_root = "src/LASER-Software/laikago_ros/rl_data/trot_05m_s/";
    // std::string tf_root = "src/LASER-Software/laikago_ros/rl_data/trot_05m_s_minObs/";
    std::string tf_root = "src/LASER-Software/laikago_ros/rl_data/boundkp500kd8/";

    // Model model;
    // Tensor observation;
    // Tensor action;

    cppflow::model model;
    cppflow::tensor observation;
    cppflow::tensor action;

    cppflow::tensor seq_in = cppflow::fill({1}, 1);
    cppflow::tensor state_h;
    cppflow::tensor state_c;

    ros::ServiceServer tf_service;

    // vec normalize params
    // int obs_len = 64; // was 48, 51, now 75 (bc +36-12) //39
    // int obs_len = 65;
    // int obs_len = 58; // remove YZ and dz from obs
    // int obs_len = 58; // remove all z and contacts from obs
    bool des_vel_in_obs = false;
    bool last_action_in_obs = true;
    int obs_len = 70; // remove all z and contacts from obs. add last action

    // bool des_vel_in_obs = true;
    // int obs_len = 66;

    int act_len = 12; // using IMPEDANCE_POS_ONLY 
    //double act_arr[12]; // act_len
    double act_arr[12] = { 0, -0.0838, -0.3, 
                        0,  0.0838, -0.3,
                        0, -0.0838, -0.3,
                        0,  0.0838, -0.3};
    vector<double> act_arr_vec;
    vector<float> obs_rms_mean;
    vector<float> obs_rms_var;
    vector<float> obs_high;
    vector<float> obs_low;
    vector<float> act_high;
    vector<float> act_low;
    bool enable_action_filter = false;
    double last_action_rl[12] = {0,0,0,0,0,0, 0,0,0,0,0,0 };
    double sim_time_obs = 0;
    double des_vel = 0;

    bool useJointPD = false;

    double joint_kp = 50;
    double joint_kd = 0.1;

    /************************** Balance controller *************************/

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
    //Vec3<double> pFeetVecCOM;
    //double fOpt[12];
    bool firstRunBalanceController = true;
};

#endif // PYBULLETINTERFACE_H