/************************************************************************

Jumping helper
- load in trajectories
- load NN
- query for actions 

************************************************************************/
#ifndef JUMPING_H
#define JUMPING_H

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

class JumpingObj
{
public:
    // TODO: add initialization for taking paths, getting modes, etc.
    JumpingObj(); 


    /************************** RL obs/act ******************************/
    /**
     * Preprocess lowState/StateEstimate to observation format for RL, following default_contacts format
     */
    void getObservation();
    void get2DObservation_hist(int Iter,  std::vector<double> init_final, std::vector<double> contactState);
    void getObservation_hist_v3(int Iter,  std::vector<double> init_final, std::vector<double> contactState);
    void getObservation_hist_v4(int Iter,  std::vector<double> init_final, std::vector<double> contactState);
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
    void computeAction(int Iter,  std::vector<double> init_final, std::vector<double> contactState); 
    void computeAction_2D(int Iter,  std::vector<double> init_final, std::vector<double> contactState); 
    void computeAction_old();
    /**
     * GENERAL set leg commands from RL (for native C++)
     */
    void generalSetLegCommandsRL();
    /**
     * Set leg commands from RL
     */
    void processAndSetLegCommandsFromRL(const laikago_msgs::RLAct& msg);


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

    void landing();
    void computeTorquesandSend_landing();

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


    /************************** Jumping Helpers  ******************************/
    void setLegControllerCommandsFromTrajIndex(int trajIdx);
    void setLegControllerCommandsFromTrajIndex_2D(int trajIdx);
    void updateLegControllerCommandsFromRL();
    void updateLegControllerCommandsFromRL_2D();
    void computeFullTorquesAndSend();
    void computeFullTorquesAndSend_constraints();
    void computeFullTorquesAndSend_constraints_v2();
    void setLegController(); 
    bool readTraj(std::string filename);
    void moveToTrajInitPos();

    int hasData = 0;
    Vec3<double> v_des_mpc;
    double yaw_rate;
    StateEstimate result;
    ControlFSMData* _controlData;
    // ConvexMPCLocomotion Cmpc;
    //double rlFootPos[8];

    // Balance controller 
    BalanceController balanceController;


    /************************** Optimal trajectories *************************/
    vector<vector<double>> x_opt; // full traj_state
    int full_opt_N; // trajectory length
    int opt_state_space = 84;
    bool USE_RL = false;
    int rl_action_idx = 0;
    int flight_idx = 800;
    // landing:
    bool runQP = false;

    int obs_len = 280;  // (28)*5 --> 2D
    double act_arr[4]; 
    int act_len = 4; //16
    double last_action_rl[4] = {0,0,0,0};
    
    vector<float> obs_rms_mean;
    vector<float> obs_rms_var;
    vector<float> obs_high;
    vector<float> obs_low;
    vector<float> act_high;
    vector<float> act_low;

    bool enable_action_filter = true;
    double act_arr_3D[12];
    
    

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
    // std::string tf_root = "src/USC-AlienGo-Software/laikago_ros/jumping/my_model2/";
    // std::string tf_root = "/home/guillaume/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/"
    //                         "usc_learning/learning/rllib/exported_model/";



    // Chuong:

    // std::string tf_root = "src/LASER-Software/laikago_ros/jumping/102822151917/"; // remove torque+add inital config
    // std::string tf_root = "src/LASER-Software/laikago_ros/jumping/110522115707/"; // remove torque+add inital config
    // std::string tf_root = "src/LASER-Software/laikago_ros/jumping/111222204147/"; // remove torque+add inital config , with load
    //  std::string tf_root = "src/LASER-Software/laikago_ros/jumping/111822143320/"; // remove torque+add inital config , with load
    // std::string tf_root = "src/LASER-Software/laikago_ros/jumping/112522231747/"; // migrate voltage

    //2D
    // std::string tf_root = "src/LASER-Software/laikago_ros/jumping/112722214809/";
    std::string tf_root = "src/LASER-Software/laikago_ros/jumping/120122185909/";
    

   
    bool normalize_obs_flag = true;

    // Model model;
    // Tensor observation;
    // Tensor action;

    cppflow::model model;
    cppflow::tensor observation;
    cppflow::tensor action;

    ros::ServiceServer tf_service;

    std::list<vector<float>> sensory_obs;

    std::vector<float> pyb_obs; 

    // QP params

    double COM_weights_stance[3] = {5, 5, 5};
    double Base_weights_stance[3] = {40, 20, 10};
    double pFeet[12], p_act[3], v_act[3], O_err[3], rpy[3],v_des[3], p_des[3], omegaDes[3];
    double se_xfb[13];
    double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];
    double b_control[6];
    double minForce = 5;
    double maxForce = 250;
    double minForces[4] = {minForce, minForce, minForce, minForce};
    double maxForces[4] = {maxForce, maxForce, maxForce, maxForce};
    Mat34<double> footFeedForwardForces;    // feedforward forces at the feet


    /************************** Leg Controller options *************************/
    Mat3<double> kpCartesian, kdCartesian;
    Mat3<double> kpJoint, kdJoint;

    // index for reference from optimization

    int BASE_POS_INDEX = 0;// + np.arange(3)
    int BASE_ORN_INDEX = 3; // + np.arange(3)
    int BASE_LINVEL_INDEX = 6; // + np.arange(3)
    int BASE_ANGVEL_INDEX = 9; // + np.arange(3)
    int JOINT_POS_INDEX = 12; // + np.arange(12)
    int JOINT_VEL_INDEX = 24; // + np.arange(12)
    //# indices for Cartesian state data
    int FOOT_POS_INDEX = 36; // + np.arange(12)
    int FOOT_VEL_INDEX = 48; // + np.arange(12)
    int FOOT_FORCE_INDEX = 60; // + np.arange(12)
    int TORQUE_INDEX = 72; // [NEW]



};

#endif // JUMPING_H