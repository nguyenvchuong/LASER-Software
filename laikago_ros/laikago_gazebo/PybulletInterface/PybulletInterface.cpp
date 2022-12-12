/************************************************************************

PybulletInterface, includes 
-communication 
-interface to MPC, BalanceController
-options for setting simulation state
-get actions from RL and set controls

-eventually may use as a basis for a gym environment class


--------------------------------
receive data from pybullet, call MPC, send back to pybullet.
Service:
- MPCService, with PybulletFullState Request, and LegCmdArray Response
Subscribe:
- reset MPC (firstRun = true)
- set new Gait num


- see usc_learning/ros_interface/run_mpc.py


************************************************************************/

#include "PybulletInterface.h"

//using namespace std;
//using namespace laikago_model;


PybulletInterface::PybulletInterface() : 
    Cmpc(0.001, 30),
    loop_rate(1000),
    // model(tf_root + "graph.pb"),
    model(tf_root + "my_model")

    // observation{model, "default_policy/observation"},
    // action{model, "default_policy/cond/Merge"}
    // new rllib version
    // observation{model, "default_policy/obs"}, 
    // action{model, "default_policy/cond/Merge"}
{
    // based on input, should choose mode (what will be listening for / using)

    //pybulletState_sub = nm.subscribe("/laikago_gazebo/pybulletFullState", 1, &PybulletInterface::PybStateCallback, this);

    // MPC
    mpc_service = nm.advertiseService("/laikago_gazebo/mpc_comm", &PybulletInterface::callMPC, this); //persistent for client, avoids reconnecting
    mpc_reset_sub = nm.subscribe("/laikago_gazebo/reset_mpc", 1, &PybulletInterface::resetMPC, this);
    mpc_gait_sub = nm.subscribe("/laikago_gazebo/set_gait", 1, &PybulletInterface::setGait, this);
    //mpc_set_rl_foot_pos_sub = nm.subscribe("/laikago_gazebo/mpc_set_rl_foot_pos", 1, &PybulletInterface::setMPCFootPos, this);
    // BalanceController
    balance_service = nm.advertiseService("/laikago_gazebo/balance_comm", &PybulletInterface::callBalanceController, this); 
    balance_reset_sub = nm.subscribe("/laikago_gazebo/reset_balance", 1, &PybulletInterface::resetBalanceController, this);

    // Rl interface
    //rl_actions_sub = nm.subscribe("/laikago_gazebo/rl_leg_cmds", 1, &PybulletInterface::setLegCommands, this);
    rl_obs_pub = nm.advertise<laikago_msgs::RLObs>("/laikago_gazebo/rl_obs", 1);
    rl_act_sub = nm.subscribe("/laikago_gazebo/rl_act", 1, &PybulletInterface::processAndSetLegCommandsFromRL, this);
    //nm.advertise<laikago_msgs::LowState>("/aliengo_gazebo/lowState/state", 1);

    // Tensorflow interface
    tf_service = nm.advertiseService("/laikago_gazebo/tf_comm", &PybulletInterface::queryTensorflowModelService, this);
    
    // Gazebo
    gazebo_pause_srv = nm.serviceClient<std_srvs::Empty>("/gazebo/pause_physics"); 
    gazebo_unpause_srv = nm.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics"); 

    python_available_sub = nm.subscribe("/laikago_gazebo/python_available", 1, &PybulletInterface::turnOn, this);
    std::cout << "before client" << std::endl;
    rl_client = nm.serviceClient<laikago_msgs::RLService>("/laikago_gazebo/rl_comm");
    std::cout << "after client" << std::endl;

    setupTensorflow();
    loadRLSpaceParams();
    
    std::cout << "init action_arr..." << std::endl;
    double init_pos[12] = { 0, -0.0838, -0.3, 
                        0,  0.0838, -0.3,
                        0, -0.0838, -0.3,
                        0,  0.0838, -0.3};
    for (int i =0; i< act_len; i++){
        //act_arr[i] = init_pos[i];
        act_arr_vec.push_back(init_pos[i]);
    }
    // init desired foot positions into action arr
    // for (int leg=0; leg<4; leg++){
    //     // foot pos
    //     for(int j=0; j<3; j++){
    //         act_arr[leg*3+j] = 0; //_controlData->_legController->data[leg].p[j];
    //     }
    // }
    std::cout << "... done action_arr" << std::endl;
    //mytime.start();
}


/****************************************************************************************************************************
******************************************************* General *************************************************************
*****************************************************************************************************************************/

/**
 * Flag callback that indicates we should start listening for (other) data.
 */
void PybulletInterface::turnOn(const std_msgs::Empty& msg )
{
    hasData = 1;
    std::cout << "turned on" << std::endl;
}

/**
 * Get RL commands, set in ROS, used as service 
 */
void PybulletInterface::getRLCommandsService()
{
    rl_srv.request.lowState = lowState;
    if(rl_client.call(rl_srv))
    {
        //unpause sim
        gazebo_unpause_srv.call(empty_srv);
        std::cout << "send leg cmds" << std::endl;

        // in a loop, update the leg cmds (pDes, kpCartesian, etc.) for 10ms (in RL chose actions every 10ms)


        setLegCommands(rl_srv.response.legCmdArray);
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

/**
 * Get leg commands from pybullet, set motor commands
 *
 * TODO: replace with pDes, kpCartesian, etc. so the torques can be automatically updated with legController->update
 * between high level action calls from RL
 * or, this can be an option flag passed in, whether to use the torques, or compute 
 */
void PybulletInterface::setLegCommands(const laikago_msgs::LegCmdArray& msg)
{
    // for(int i = 0; i < 12; i++){
    //     lowCmd.motorCmd[i].torque = msg.motorCmd[i];
    // }
    //sendServoCmd();

    // set high level commands, will be used several times while waiting for next high level command from pybullet
    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 3; j++){
            _controlData->_legController->commands[i].pDes[j] = msg.legCmds[i].pDes[j]; //pDesLeg;
            _controlData->_legController->commands[i].vDes[j] = msg.legCmds[i].vDes[j];
            _controlData->_legController->commands[i].kpCartesian(j,j) = msg.legCmds[i].kpCartesianDiag[j];
            _controlData->_legController->commands[i].kdCartesian(j,j) = msg.legCmds[i].kdCartesianDiag[j];
            _controlData->_legController->commands[i].feedforwardForce[j] = msg.legCmds[i].feedforwardForce[j];
        }
    }
    _controlData->_legController->updateCommand();
    
    hasData = 1;
}


/** 
 * Sets the following information:
 *   struct StateEstimate {
 *       Vec3<double> position;
 *       Quat<double> orientation;
 *       Vec3<double> rpy;
 *       RotMat<double> rBody;
 *       Vec3<double> vBody;
 *       Vec3<double> omegaBody;
 *       Vec3<double> vWorld;
 *       Vec3<double> omegaWorld;
 *       Vec4<double> contactEstimate;
 *       Vec3<double> aBody, aWorld;
 *   }
 *   MotorStates: lowState.motorState[].{pos/vel/torque}
 *   // desired body vel and yaw_rate
 *   Vec3<double> v_des
 *   double yaw_rate
 */
void PybulletInterface::setStateFromPybullet(const laikago_msgs::PybulletFullState& pybState)
{
    // quick hack, should be with Eigen mapping
    for (int i = 0; i < 3; i++){
        result.position[i] = pybState.position[i];
        result.rpy[i] = pybState.rpy[i];
        result.vBody[i] = pybState.vBody[i];
        result.omegaBody[i] = pybState.omegaBody[i];
        result.vWorld[i] = pybState.vWorld[i];
        result.omegaWorld[i] = pybState.omegaWorld[i];
        // v_des
        v_des_mpc[i] = pybState.v_des[i];
        // also set extras not currently used
        result.aBody[i] = 0;
        result.aWorld[i] = 0;
    }
    // orientation
    for (int i = 0; i < 4; i++){
        result.orientation[i] = pybState.orientation[i];
    }
    // rotation matrix (note, column major order for eigen )
    int rBodyIndex = 0;
    for (int i = 0; i < 3; i++){
        for (int j = 0; j < 3; j++){
            result.rBody(i,j) = pybState.rBody[rBodyIndex];
            rBodyIndex++;
        }
    }
    // loop through motor states
    for (int i =0; i < 12; i++){
        lowState.motorState[i].mode = pybState.motorState[i].mode;
        lowState.motorState[i].position = pybState.motorState[i].position;
        lowState.motorState[i].velocity = pybState.motorState[i].velocity;
        lowState.motorState[i].torque = pybState.motorState[i].torque;
    }
    yaw_rate = pybState.yaw_rate_des;
    hasData = 1;

    // check if desired foot positions involved, and set for Cmpc

    // this updates leg q,qd,tau from motorState 
    _controlData->_legController->updateData();
    // replace the stateEstimator with the data from pybullet
    _controlData->_stateEstimator->setStateEstimate(result);
}

/**
 * Set LegCmdArray from current (or computed) ROS state, usually to be sent to pybullet
 */
void PybulletInterface::setResponseLegCmdArray(laikago_msgs::LegCmdArray& legCmdArray)
{
    //first update to make sure it's current
    _controlData->_legController->updateCommandNoSend();
    // loop through legs
    for (int i = 0; i < 4; i++){
        for (int j = 0; j<3; j++){
            legCmdArray.legCmds[i].pDes[j] = _controlData->_legController->commands[i].pDes[j];
            legCmdArray.legCmds[i].vDes[j] = _controlData->_legController->commands[i].vDes[j];
            legCmdArray.legCmds[i].kpCartesianDiag[j] = _controlData->_legController->commands[i].kpCartesian(j,j);
            legCmdArray.legCmds[i].kdCartesianDiag[j] = _controlData->_legController->commands[i].kdCartesian(j,j);
            legCmdArray.legCmds[i].feedforwardForce[j] = _controlData->_legController->commands[i].feedforwardForce[j];
            legCmdArray.motorCmd[i*3+j] = lowCmd.motorCmd[i*3+j].torque;
        }
    }
}

/****************************************************************************************************************************
************************************************* Balance Controller ********************************************************
*****************************************************************************************************************************/
/**
 * Call Balance Controller from pybullet
 */
bool PybulletInterface::callBalanceController(laikago_msgs::MPCServiceRequest& msg, laikago_msgs::MPCServiceResponse& response)
{
    // set state
    setStateFromPybullet(msg.pybState);
    // run Balance controller
    runBalanceController();
    // set response leg commands
    setResponseLegCmdArray(response.legCmdArray);

    return true;
}

/**
 * Run actual QP for balance controller
 */
void PybulletInterface::runBalanceController()
{
    if (firstRunBalanceController){
        // if first run, set desired position and rpy to be the current one
        //  -if update every time, will drift

        // update desired position & rpy
        for(int i = 0; i < 3; i++){
            p_des[i] = _controlData->_stateEstimator->getResult().position(i); 
            v_des[i] = 0;
            omegaDes[i] = 0;
            rpy[i] = 0;
        }
        rpy[2] = _controlData->_stateEstimator->getResult().rpy(2);
        p_des[2] = 0.3;
        firstRunBalanceController = false;
    }   

    // p_des[0] = 0;
    // p_des[1] = 0;
    // rpy[2] = 0;
    //std::cout << "des x y" << p_des[0] << " " <<p_des[1] << std::endl;

    // se_xfb holds all relevant states for simplified model for QP
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
    }
    //_controlData->_legController->updateCommandNoSend();

    // loop through legController commands to create LegCmdArray message (this should be a function)
    // for (int i = 0; i < 4; i++){
    //     for (int j = 0; j<3; j++){
    //         response.legCmdArray.legCmds[i].pDes[j] = _controlData->_legController->commands[i].pDes[j];
    //         response.legCmdArray.legCmds[i].vDes[j] = _controlData->_legController->commands[i].vDes[j];
    //         response.legCmdArray.legCmds[i].kpCartesianDiag[j] = _controlData->_legController->commands[i].kpCartesian(j,j);
    //         response.legCmdArray.legCmds[i].kdCartesianDiag[j] = _controlData->_legController->commands[i].kdCartesian(j,j);
    //         response.legCmdArray.legCmds[i].feedforwardForce[j] = _controlData->_legController->commands[i].feedforwardForce[j];
    //         response.legCmdArray.motorCmd[i*3+j] = lowCmd.motorCmd[i*3+j].torque;
    //     }
    // }
}


/**
 * Reset, so next call to runBalanceController will try to keep base at new x,y position
 *  -without this, body drifts and then falls down
 */
void PybulletInterface::resetBalanceController(const std_msgs::Empty& msg)
{
    firstRunBalanceController = true;
}
/****************************************************************************************************************************
********************************************************* MPC ***************************************************************
*****************************************************************************************************************************/

/*
* Take in robot state from pybullet
* compute MPC actions
*/
// formats: (laikago_msgs::PybulletFullState& msg, laikago_msgs::LegCmdArray& response)
bool PybulletInterface::callMPC(laikago_msgs::MPCServiceRequest& msg, laikago_msgs::MPCServiceResponse& response)
{
    // set _controlData state
    setStateFromPybullet(msg.pybState);
    // whether RL is active, and to choose feet position from RL
    if(msg.pybState.rlFlag){
        //double rlFootPos[8];
        for(int i = 0; i<8; i++){
            rlFootPos[i] = msg.pybState.rlFootPos[i];
        }
        //Cmpc.setRLFootPositions(msg.pybState.rlFootPos);
        Cmpc.setRLFootPositions(rlFootPos);
        //std::cout << " got rl Flag " << std::endl;
    }

    // this updates leg q,qd,tau from motorState 
    _controlData->_legController->updateData();
    // replace this with the data from pybullet
    _controlData->_stateEstimator->setStateEstimate(result);
    // set desired body vel (v_des) and yaw_rate
    _controlData->_desiredStateCommand->setStateCommands(v_des_mpc, yaw_rate);

    // set gait (trot)
    Cmpc.setGaitNum(gaitNum);
    // run MPC
    Cmpc.run(*_controlData);

    // set response with leg commands
    setResponseLegCmdArray(response.legCmdArray);

    return true;

}


void PybulletInterface::resetMPC(const std_msgs::Empty& msg)
{
    Cmpc.reset(); // verify this is enough to reset
    std::cout << "reset MPC" << std::endl;
}

void PybulletInterface::setGait(const std_msgs::Int32& msg)
{
    gaitNum = msg.data;
    Cmpc.setGaitNum(gaitNum);
}

/**
 * This will probably be called as part of the pybullet full state, with desired feet positions
 */
// void PybulletInterface::setMPCFootPos(const std_msgs::Float32& msg)
// {
//     Cmpc.setRLFootPositions(msg.data);
// }
// void PybulletInterface::setMPCFootPos(const std_msgs::Int32& msg)
// {
//     Cmpc.setRLFootPositions(msg.data);
// }



/****************************************************************************************************************************
*********************************************** RL Pre and post processing **************************************************
*****************************************************************************************************************************/

/**
 * Preprocess lowState to observation format for RL,follow default_contacts format
 */
void PybulletInterface::getObservation()
{
    
    auto& seResult = _controlData->_stateEstimator->getResult();
    // rl_obs will have the observation
    // joints: lowState.motorState[i].{position, velocity, torque}
    for(int i = 0; i < 12; i++){
        rl_obs.data[i] = lowState.motorState[i].position;
        rl_obs.data[i+12] = lowState.motorState[i].velocity;
        // rl_obs.data[i+24] = lowState.motorState[i].torque;
    }
    int idx = 24;
    // int idx = 36;
    // orientation
    rl_obs.data[idx]   = seResult.orientation[1];
    rl_obs.data[idx+1] = seResult.orientation[2];
    rl_obs.data[idx+2] = seResult.orientation[3];
    rl_obs.data[idx+3] = seResult.orientation[0];
    idx += 4;

    // y position
    rl_obs.data[idx] = seResult.position[1];
    idx += 1;

    // z position
    rl_obs.data[idx] = seResult.position[2];
    idx += 1;

    // // contact bools
    // for (int i = 0; i < 4; i++){
    //     if (lowState.footForce[i] > 0){
    //         rl_obs.data[idx+i] = 1; // positive force, so in contact
    //     }
    //     else{
    //         rl_obs.data[idx+i] = 0; // in air
    //     }
    // }
    // idx += 4;

    // world frame velocity (want world frame for pybullet)
    Vec3<double> wFrameVel = seResult.rBody.transpose() * seResult.vBody;
    for (int i = 0; i < 3; i++){
        rl_obs.data[idx+i] = wFrameVel[i];
    }
    idx += 3;

    // angular velocity
    for (int i = 0; i < 3; i++){
        rl_obs.data[idx+i] = seResult.omegaBody[i];
    }

    idx += 3;
    // add foot positions/ velocities, and desired foot positions (last action)
    
    for (int leg=0; leg<4; leg++){
        // foot pos
        for(int j=0; j<3; j++){
            rl_obs.data[idx+j] = _controlData->_legController->data[leg].p[j];
        }
        idx+=3;
        // foot vel
        for(int j=0; j<3; j++){
            rl_obs.data[idx+j] = _controlData->_legController->data[leg].v[j];
        }
        idx+=3;
    }

    // contact bools
    for (int i = 0; i < 4; i++){
        if (lowState.footForce[i] > 0){
            rl_obs.data[idx+i] = 1; // positive force, so in contact
        }
        else{
            rl_obs.data[idx+i] = 0; // in air
        }
    }
    idx += 4;

    if (des_vel_in_obs){
        rl_obs.data[idx] = des_vel;
        idx++;
    }

    // sim time in obs (check if using!)
    rl_obs.data[idx] = sim_time_obs;

    // this is also in _legController pDes    
    // for (int i=0; i<act_len; i++){
    //     rl_obs.data[idx+i] = act_arr_vec[i];
    // }
    // for (int leg=0; leg<4; leg++){
    //     for (int j = 0; j<3; j++){
    //         rl_obs.data[idx+3*leg+j] = _controlData->_legController->commands[leg].pDes[j];
    //     }
    // }
    
    // std::cout << "======================================================================" << std::endl;
    // std::cout << "checking obs len... " << idx << std::endl;
    // for (int i = 0; i < obs_len ; i++){
    //     std::cout << i << " " << rl_obs.data[i] << std::endl;
    // }

}
/**
 * Get info from sensors only (NO cheater states)
 */
void PybulletInterface::getRealObservation()
{
    
    auto& seResult = _controlData->_stateEstimator->getResult();
    // rl_obs will have the observation
    // joints: lowState.motorState[i].{position, velocity, torque}
    for(int i = 0; i < 12; i++){
        rl_obs.data[i] = lowState.motorState[i].position;
        rl_obs.data[i+12] = lowState.motorState[i].velocity;
        // rl_obs.data[i+24] = lowState.motorState[i].torque;
    }
    int idx = 24;
    // int idx = 36;
    rl_obs.data[idx]   = seResult.orientation[1];
    rl_obs.data[idx+1] = seResult.orientation[2];
    rl_obs.data[idx+2] = seResult.orientation[3];
    rl_obs.data[idx+3] = seResult.orientation[0];
    idx += 4;
    // std::cout << "checking orientation " << seResult.orientation << std::endl;

    // y position
    rl_obs.data[idx] = seResult.position[1];
    idx += 1;

    // z position
    rl_obs.data[idx] = seResult.position[2];
    idx += 1;

    // std::cout << "checking position " << seResult.position << std::endl;
    // std::cout << "checking vBody " << seResult.vBody << std::endl;

    // // contact bools
    // for (int i = 0; i < 4; i++){
    //     if (lowState.footForce[i] > 0){
    //         rl_obs.data[idx+i] = 1; // positive force, so in contact
    //     }
    //     else{
    //         rl_obs.data[idx+i] = 0; // in air
    //     }
    // }
    // idx += 4;

    // world frame velocity (want world frame for pybullet)
    Vec3<double> wFrameVel = seResult.rBody.transpose() * seResult.vBody;
    for (int i = 0; i < 3; i++){
        rl_obs.data[idx+i] = wFrameVel[i];
    }
    idx += 3;

    // angular velocity
    for (int i = 0; i < 3; i++){
        rl_obs.data[idx+i] = seResult.omegaBody[i];
    }

    idx += 3;
    // add foot positions/ velocities, and desired foot positions (last action)
    
    for (int leg=0; leg<4; leg++){
        // foot pos
        for(int j=0; j<3; j++){
            rl_obs.data[idx+j] = _controlData->_legController->data[leg].p[j];
        }
        idx+=3;
        // foot vel
        for(int j=0; j<3; j++){
            rl_obs.data[idx+j] = _controlData->_legController->data[leg].v[j];
        }
        idx+=3;
    }

    //_controlData._stateEstimator->setContactPhase(se_contactState);
    // Vec4<double> se_contactState(0,0,0,0);
    // contact bools
    for (int i = 0; i < 4; i++){
        if (lowState.footForce[i] > 0){
            rl_obs.data[idx+i] = 1; // positive force, so in contact
            // se_contactState[i] = 1;
        }
        else{
            rl_obs.data[idx+i] = 0; // in air
            // se_contactState[i] = 0;
        }
    }
    idx += 4;
    // _controlData->_stateEstimator->setContactPhase(se_contactState);

    if (des_vel_in_obs){
        rl_obs.data[idx] = des_vel;
        idx++;
    }

    // sim time in obs (check if using!)
    rl_obs.data[idx] = sim_time_obs;

    // this is also in _legController pDes    
    // for (int i=0; i<act_len; i++){
    //     rl_obs.data[idx+i] = act_arr_vec[i];
    // }
    // for (int leg=0; leg<4; leg++){
    //     for (int j = 0; j<3; j++){
    //         rl_obs.data[idx+3*leg+j] = _controlData->_legController->commands[leg].pDes[j];
    //     }
    // }
    
    // std::cout << "======================================================================" << std::endl;
    // std::cout << "checking obs len... " << idx << std::endl;
    // for (int i = 0; i < obs_len ; i++){
    //     std::cout << i << " " << rl_obs.data[i] << std::endl;
    // }

}

/**
 * No z info
 */
void PybulletInterface::getMinimalObservation()
{
    
    auto& seResult = _controlData->_stateEstimator->getResult();
    // rl_obs will have the observation
    // joints: lowState.motorState[i].{position, velocity, torque}
    for(int i = 0; i < 12; i++){
        rl_obs.data[i] = lowState.motorState[i].position;
        rl_obs.data[i+12] = lowState.motorState[i].velocity;
        // rl_obs.data[i+24] = lowState.motorState[i].torque;
    }
    int idx = 24;
    // int idx = 36;
    rl_obs.data[idx]   = seResult.orientation[1];
    rl_obs.data[idx+1] = seResult.orientation[2];
    rl_obs.data[idx+2] = seResult.orientation[3];
    rl_obs.data[idx+3] = seResult.orientation[0];
    idx += 4;
    // std::cout << "checking orientation " << seResult.orientation << std::endl;
    // std::cout << "checking position " << seResult.position << std::endl;
    // std::cout << "checking vBody " << seResult.vBody << std::endl;


    // world frame velocity (want world frame for pybullet)
    Vec3<double> wFrameVel = seResult.rBody.transpose() * seResult.vBody;
    for (int i = 0; i < 2; i++){
        rl_obs.data[idx+i] = wFrameVel[i];
    }
    idx += 2;

    // angular velocity
    for (int i = 0; i < 3; i++){
        rl_obs.data[idx+i] = seResult.omegaBody[i];
    }

    idx += 3;
    // add foot positions/ velocities, and desired foot positions (last action)
    
    for (int leg=0; leg<4; leg++){
        // foot pos
        for(int j=0; j<3; j++){
            rl_obs.data[idx+j] = _controlData->_legController->data[leg].p[j];
        }
        idx+=3;
        // foot vel
        for(int j=0; j<3; j++){
            rl_obs.data[idx+j] = _controlData->_legController->data[leg].v[j];
        }
        idx+=3;
    }

    if (des_vel_in_obs){
        rl_obs.data[idx] = des_vel;
        idx++;
    }

    if (last_action_in_obs){
        for(int j=0; j<12; j++){
            rl_obs.data[idx] = last_action_rl[j];
            idx++;
        }
    }

    // sim time in obs (check if using!)
    rl_obs.data[idx] = sim_time_obs;

}



/**
 * Set simulation time remaining in obs
 */
void PybulletInterface::setSimTimeObs(double t){
    sim_time_obs = t;
}

/**
 * Set desired velocity  in obs
 */
void PybulletInterface::setDesVelObs(double vel){
    des_vel = vel;
}

/**
 * Place velocities in vector (linear [body, world], angular [body,world])
 */
void PybulletInterface::getVelocities(std::vector<double>& vel){
    // data
    auto& seResult = _controlData->_stateEstimator->getResult();
    // idx
    int vel_idx = 0;
    // body frame velocity 
    for (int i = 0; i < 3; i++){
        vel[vel_idx+i] = seResult.vBody[i];
    }
    vel_idx += 3;
    // world frame velocity (want world frame for pybullet)
    Vec3<double> wFrameVel = seResult.rBody.transpose() * seResult.vBody;
    for (int i = 0; i < 3; i++){
        vel[vel_idx+i] = wFrameVel[i];
    }
    vel_idx += 3;
    // angular velocity body
    for (int i = 0; i < 3; i++){
        vel[vel_idx+i] = seResult.omegaBody[i];
    }
    vel_idx += 3;
    // angular velocity world
    for (int i = 0; i < 3; i++){
        vel[vel_idx+i] = seResult.omegaWorld[i];
    }
}

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
        contact booleans 
 */
void PybulletInterface::getFullState(std::vector<double>& full_state){
    // data
    auto& seResult = _controlData->_stateEstimator->getResult();
    // idx
    int idx = 0;
    // body pos
    for (int i = 0; i < 3; i++){
        full_state[idx+i] = seResult.position[i];
    }
    idx += 3;
    // body rpy
    for (int i = 0; i < 3; i++){
        full_state[idx+i] = seResult.rpy[i];
    }
    idx += 3;
    // body frame linear velocity 
    for (int i = 0; i < 3; i++){
        full_state[idx+i] = seResult.vBody[i];
    }
    idx += 3;
    // body frame angular velocity
    for (int i = 0; i < 3; i++){
        full_state[idx+i] = seResult.omegaBody[i];
    }
    idx += 3;
    for(int i = 0; i < 12; i++){
        full_state[idx+i] = lowState.motorState[i].position;
        full_state[idx+i+12] = lowState.motorState[i].velocity;
        full_state[idx+i+24] = lowState.motorState[i].torque;
    }
    idx += 36;
    // foot positions and velocities 
    for (int leg=0; leg<4; leg++){
        // foot pos
        for(int j=0; j<3; j++){
            full_state[idx+j] = _controlData->_legController->data[leg].p[j];
        }
        idx+=3;
        // foot vel
        for(int j=0; j<3; j++){
            full_state[idx+j] = _controlData->_legController->data[leg].v[j];
        }
        idx+=3;
    }
    // contact bools
    //std::cout << "==" << std::endl;
    for (int i = 0; i < 4; i++){
        // if (lowState.footForce[i] > 0){
        //     full_state[idx+i] = 1; // positive force, so in contact
        // }
        // else{
        //     full_state[idx+i] = 0; // in air
        // }
        full_state[idx+i] = lowState.footForce[i];
        //std::cout << _controlData->_legController->commands[i].feedforwardForce << std::endl;
    }
}


/**
 * Send raw obs to pybullet
 */
void PybulletInterface::sendObservation()
{
    // construct obs
    getObservation();
    // publish
    rl_obs_pub.publish(rl_obs);
}


/**
 * Process observation like in VecNormalize, need mean and std dev (read in from files)
 */
void PybulletInterface::normalizeObservation(std::vector<float>& obs)
{
    for(int i = 0; i < obs_len; i++){
        // normalize
        // clip to limits
        if (rl_obs.data[i] > obs_high[i]) //(rl_obs.data[i] > obs_high[i])
            rl_obs.data[i] = obs_high[i];
        else if (rl_obs.data[i] < obs_low[i])
            rl_obs.data[i] = obs_low[i];
        
        // (obs - self.obs_rms.mean) / np.sqrt(self.obs_rms.var + self.epsilon),
        obs[i] = (rl_obs.data[i] - obs_rms_mean[i] ) / sqrt( obs_rms_var[i] + 1e-8 );
        // clip to limits
        // if (obs[i] > obs_high[i]) //(rl_obs.data[i] > obs_high[i])
        //     obs[i] = obs_high[i];
        // else if (obs[i] < obs_low[i])
        //     obs[i] = obs_low[i];
        // else
        //     obs[i] = rl_obs.data[i];
    }
}

/**
 * full RL method: get obs, scale, send to NN, apply leg commands
 */
void PybulletInterface::computeAction()
{
    // construct obs
    // getObservation();
    // getRealObservation();
    getMinimalObservation();
    // normalize observation
    // std::cout << "norm obs" << std::endl;
    std::vector<float> obs(obs_len);
    normalizeObservation(obs);

    // get action from policy network
    // set observation
    // std::cout << "set data" << std::endl;
    // observation.set_data(obs);
    // run model and set action
    // std::cout << "run model" << std::endl;
    // std::cout << "======================================================================" << std::endl;
    // std::cout << "checking obs len... "  << std::endl;
    // for (int i = 0; i < obs_len ; i++){
    //     std::cout << i << " " << obs[i]  << std::endl;
    // }
    // std::cout << "checking act... " << std::endl;
    // for (int i = 0; i < act_len ; i++){
    //     std::cout << i << " " << action[i] << << std::endl;
    // }
    // model.run({&observation}, action);

    observation = cppflow::tensor(obs, {1, 1, obs_len});

    auto action_out = model(
        {{"serving_default_inputs:0", observation},
        {"serving_default_seq_in:0", seq_in},
        {"serving_default_h:0", state_h},
        {"serving_default_c:0", state_c}
        },
        {"StatefulPartitionedCall:0", "StatefulPartitionedCall:1", "StatefulPartitionedCall:2", "StatefulPartitionedCall:3"}
    );

    action = action_out[0];
    state_c = action_out[1];
    state_h = action_out[2];

    // std::cout << "post model run" << std::endl;
    act_arr_vec.clear();

    // scale action:
    int i = 0;
    for (float f : action.get_data<float>()) {
        float a = f;
        // clip
        if (a > 1)
            a = 1.0;
        else if (a < -1)
            a = -1.0;

        last_action_rl[i] = a;

        if (enable_action_filter){
            double lam = 0.9;
            double temp_a = a;
            a = lam*a + (1-lam) * last_action_rl[i];
            last_action_rl[i] = temp_a;
        }
        // scale
        //std::cout << "act_low[i] " << act_low[i] << " a " << a << " act_high[i] " << act_high[i] << std::endl;
        a = act_low[i] + 0.5 * (a+1) * (act_high[i] - act_low[i]);
        //std::cout << "new a: " << a << std::endl;
        // set RLAct array
        rl_act.data[i] = a;
        act_arr[i] = a;
        act_arr_vec.push_back((double) a);
        i++;
    }
    // sets legCommands
    //processAndSetLegCommandsFromRL(rl_act);
    generalSetLegCommandsRL();

}


/**
 * Scale action to ranges for pDes and Fn
 */
// void PybulletInterface::scaleAction()
// {
//     // set rl_act data to ranges
//     processAndSetLegCommandsFromRL(rl_act);
// }

/**
 * GENERAL set RL actions [-1,1] to leg commands, for general action space, depending on act_len
 */
void PybulletInterface::generalSetLegCommandsRL()
{

    // assume already scaled, so just set pDes, etc.
    auto& seResult = _controlData->_stateEstimator->getResult();
    Vec3<double> feedforwardForce;
    int forcesIdx = 12;

    // check contact bools, only apply force if in contact
    double inContact[4];
    for (int i = 0; i < 4; i++){
        if (lowState.footForce[i] > 0){
            inContact[i] = 1; // positive force, so in contact
        }
        else{
            inContact[i] = 0; // in air
        }
    }

    // set high level commands, may be used several times while waiting for next high level command from pybullet
    for (int i = 0; i < 4; i++){
        // rotate world frame force into body frame
        if (inContact[i]){
            // IMPEDANCE_POS_ONLY action space 
            if (act_len == 12){
                feedforwardForce << 0, 0, 0;
            }
            else if (act_len == 16){
                // IMPEDANCE action space
                feedforwardForce << 0, 0, act_arr[forcesIdx+i];
            }
            else {
                // IMPEDANCE_3D_FORCE action space
                feedforwardForce << act_arr[forcesIdx+i*3], act_arr[forcesIdx+i*3+1], act_arr[forcesIdx+i*3+2];
            }
            feedforwardForce = seResult.rBody.transpose() * feedforwardForce; //msg.data[forcesIdx+i];
        }
        else {
             feedforwardForce << 0, 0, 0;
        }
        for (int j = 0; j < 3; j++){
            _controlData->_legController->commands[i].pDes[j] = act_arr[i*3+j]; 
            _controlData->_legController->commands[i].vDes[j] = 0; 
            _controlData->_legController->commands[i].kpCartesian(j,j) = 500;//1000;//800; //500;
            _controlData->_legController->commands[i].kdCartesian(j,j) = 8; // 30; //10; 
            // _controlData->_legController->commands[i].kpCartesian(j,j) = 700;
            // _controlData->_legController->commands[i].kdCartesian(j,j) = 12;
            _controlData->_legController->commands[i].feedforwardForce[j] = feedforwardForce[j]; 
        }
    }

    // _controlData->_legController->updateCommand();
    _controlData->_legController->updateCommandNoSend();
    
    hasData = 1;

}

void PybulletInterface::sendCommand(){
    if (useJointPD){
        for (int i = 0; i<4; i++){
            for (int j = 0; j<3; j++){
                lowCmd.motorCmd[i*3+j].position = act_arr[i*3+j];
                lowCmd.motorCmd[i*3+j].velocity = 0;
                lowCmd.motorCmd[i*3+j].torque = 0;
                lowCmd.motorCmd[i*3+j].positionStiffness = joint_kp;
                lowCmd.motorCmd[i*3+j].velocityStiffness = joint_kd;
                // set other quantities to 0
                _controlData->_legController->commands[i].pDes[j] = 0;
                _controlData->_legController->commands[i].vDes[j] = 0;
                _controlData->_legController->commands[i].kpCartesian(j,j) = 0;
                _controlData->_legController->commands[i].kdCartesian(j,j) = 0;
                _controlData->_legController->commands[i].feedforwardForce[j]  = 0;
            }
        }
        sendServoCmd();
    }
    else{

        _controlData->_legController->updateCommandNoSend();
        // add in IK 
        // for (int i = 0; i<4; i++){
        //     Vec3<double> qDes, tau;
        //     auto pDes = _controlData->_legController->commands[i].pDes;
        //     computeInverseKinematics(_controlData->_legController->_quadruped, pDes, i, &qDes);
        //     tau = joint_kp * (qDes - _controlData->_legController->data[i].q) + 
        //           joint_kd * ( - _controlData->_legController->data[i].qd);
        //     for (int j = 0; j<3; j++){
        //         // lowCmd.motorCmd[i*3+j].position = qDes[j];
        //         // lowCmd.motorCmd[i*3+j].velocity = 0;
        //         // lowCmd.motorCmd[i*3+j].torque = 0;
        //         // lowCmd.motorCmd[i*3+j].positionStiffness = 100;//joint_kp;
        //         // lowCmd.motorCmd[i*3+j].velocityStiffness = 1;//joint_kd;
        //         lowCmd.motorCmd[i*3+j].torque += tau[j];
        //     }
        // } 
        sendServoCmd();

        // before was just this
        // _controlData->_legController->updateCommand();
    }
}

/**
 * Postprocess raw actions that are in [-1,1] to leg commands, this for IMPEDANCE action space
 */
void PybulletInterface::processAndSetLegCommandsFromRL(const laikago_msgs::RLAct& msg)
{

    // assume already scaled, so just set pDes, etc.
    auto& seResult = _controlData->_stateEstimator->getResult();
    Vec3<double> feedforwardForce;
    int forcesIdx = 12;

    // check contact bools, only apply force if in contact
    double inContact[4];
    for (int i = 0; i < 4; i++){
        if (lowState.footForce[i] > 0){
            inContact[i] = 1; // positive force, so in contact
        }
        else{
            inContact[i] = 0; // in air
        }
    }

    // set high level commands, may be used several times while waiting for next high level command from pybullet
    for (int i = 0; i < 4; i++){
        // rotate world frame force into body frame
        if (inContact[i]){
            //Vec3<double> feedforwardForce(0, 0, msg.data[forcesIdx+i]);
            feedforwardForce << 0, 0, msg.data[forcesIdx+i];
            feedforwardForce = seResult.rBody.transpose() * feedforwardForce; //msg.data[forcesIdx+i];
        }
        else {
             feedforwardForce << 0, 0, 0;
        }
        for (int j = 0; j < 3; j++){
            _controlData->_legController->commands[i].pDes[j] = msg.data[i*3+j]; //legCmds[i].pDes[j]; //pDesLeg;
            _controlData->_legController->commands[i].vDes[j] = 0; //msg.legCmds[i].vDes[j];
            _controlData->_legController->commands[i].kpCartesian(j,j) = 500;//msg.legCmds[i].kpCartesianDiag[j];
            _controlData->_legController->commands[i].kdCartesian(j,j) = 10; //msg.legCmds[i].kdCartesianDiag[j];
            _controlData->_legController->commands[i].feedforwardForce[j] = feedforwardForce[j]; //msg.legCmds[i].feedforwardForce[j];
        }
    }

    _controlData->_legController->updateCommand();
    
    hasData = 1;

}


/****************************************************************************************************************************
*********************************************** Tensorflow loading/sending **************************************************
*****************************************************************************************************************************/
/**
 * Setup tensorflow graphs and placeholders.
 */
void PybulletInterface::setupTensorflow()
{
    std::cout << "restore TF model at " << tf_root << std::endl;
    //model.init();
    // model.restore("/home/guillaume/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/"
    //                 "usc_learning/learning/rllib/exported/my_model");
    // model.restore(tf_root + "my_model");
    model = cppflow::model(tf_root + "my_model");

    std::cout << "...done" << std::endl;
}

/**
 * Query tensorflow service as test from pybullet.
 */
bool PybulletInterface::queryTensorflowModelService(laikago_msgs::TFServiceRequest& msg, laikago_msgs::TFServiceResponse& response)
{
    // set observation
    //observation.set_data(msg.rlObs.data.cast<float>());
    std::vector<float> data(obs_len);
    for (int i = 0; i < obs_len; i++){
        data[i] = msg.rlObs.data[i];
    }
    // observation.set_data(data);
    observation = cppflow::tensor(data);
    // run model and set action
    // model.run({&observation}, action);
    auto action = model(
        {{"serving_default_inputs:0", observation},
        {"serving_default_seq_in:0", seq_in},
        {"serving_default_h:0", state_h},
        {"serving_default_c:0", state_c}
        },
        {"StatefulPartitionedCall:0", "StatefulPartitionedCall:1", "StatefulPartitionedCall:2", "StatefulPartitionedCall:3"}
    )[0];
    // set action in response and send
    //response.rlAct.data = action;
    int idx = 0;
    for (float f : action.get_data<float>()) {
        response.rlAct.data[idx] = f;
        idx++;
    }

    return true;
}

/**
 * Load VecNormalize params (mean/var) and observation/action high/low
 */
bool PybulletInterface::loadRLSpaceParams()
{
    // Read observation params 
    std::ifstream vFile;
    // vFile.open("/home/guillaume/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/"
    //                 "usc_learning/learning/rllib/exported/vecnorm_params.csv");
    vFile.open(tf_root + "vecnorm_params.csv");
    // If cannot open the file, report an error
    if(!vFile.is_open())
    {
        std::cout << "Could not open vecnorm params file" << std::endl;
        return false;
    }
    cout << "Reading vecnorm params..." << endl;

    string line;
    int index = 0;

    while(getline(vFile, line))
    {
        // Line[0] has the observation mean
        if(index == 0)
        {
            stringstream ss(line);
            float val;
            while(ss >> val)
            {
                obs_rms_mean.push_back(val);
                //cout << "obs_rms_mean " << val << endl;
                if(ss.peek() == ',') ss.ignore();
            }
        }
        // Line[1] has the observation variance
        if(index == 1)
        {
            stringstream ss(line);
            float val;
            while(ss >> val)
            {
                obs_rms_var.push_back(val);
                //cout << "obs_rms_var " << val << endl;
                if(ss.peek() == ',') ss.ignore();
            }
        }
        // Line[2] has the observation high 
        if(index == 2)
        {
            stringstream ss(line);
            float val;
            while(ss >> val)
            {
                obs_high.push_back(val);
                //cout << "obs_high " << val << endl;
                if(ss.peek() == ',') ss.ignore();
            }
        }
        // Line[3] has the observation low
        if(index == 3)
        {
            stringstream ss(line);
            float val;
            while(ss >> val)
            {
                obs_low.push_back(val);
                //cout << "obs_low " << val << endl;
                if(ss.peek() == ',') ss.ignore();
            }
        }

        index++;
    }
    vFile.close();

    // Read the action space file
    std::ifstream aFile;
    // aFile.open("/home/guillaume/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/"
    //                 "usc_learning/learning/rllib/exported/action_space.csv");
    aFile.open(tf_root + "action_space.csv");
    // If cannot open the file, report an error
    if(!aFile.is_open())
    {
        std::cout << "Could not open action space file" << std::endl;
        return false;
    }
    cout << "Reading action space params..." << endl;
    index = 0;

     while(getline(aFile, line))
    {
        // Line[0] has the action upper range
        if(index == 0)
        {
            stringstream ss(line);
            float val;
            while(ss >> val)
            {
                act_high.push_back(val);
                //cout << "val high " << val << endl;
                if(ss.peek() == ',') ss.ignore();
            }
        }
        // Line[1] has the action lower range
        if(index == 1)
        {
            stringstream ss(line);
            float val;
            while(ss >> val)
            {
                act_low.push_back(val);
                //cout << "val low " << val << endl;
                if(ss.peek() == ',') ss.ignore();
            }
        }
        index++;
    }
    aFile.close();

    // read in joint PD gains
    if (useJointPD){
        aFile.open(tf_root + "joint_gains.csv");
        if(!aFile.is_open())
        {
            std::cout << "Could not open joint gains file" << std::endl;
            return false;
        }
        cout << "Reading joint gains..." << endl;
        index = 0;

        getline(aFile, line);
        cout << "line: " << line << std::endl;
        // stringstream ss(line);
        // double val;

        // ss >> val;
        // joint_kp = val;
        // ignore comma
        // ss >> val;
        // ss.ignore();
        // getline(aFile, line);
        // ss << line;
        // ss >> val;
        // joint_kd = val;

        joint_kp = stod(line);
        getline(aFile, line);
        joint_kd = stod(line);

        cout << "New jont kp, kd " << joint_kp << " " << joint_kd << endl;

        aFile.close();
    }

    cout << "...done" << endl;
    return true;
}