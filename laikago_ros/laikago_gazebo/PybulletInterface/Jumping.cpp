/************************************************************************
Jumping class
************************************************************************/

#include "Jumping.h"

// using namespace std;
// using namespace laikago_model;

JumpingObj::JumpingObj() :

                           // Cmpc(0.001, 30),
                           loop_rate(1000),
                           // model("/home/guillaume/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/"
                           //          "usc_learning/learning/rllib/exported/graph.pb"),
                           // model(tf_root + "graph.pb"),
                           model(tf_root + "my_model")
// SAC (or PPO) rllib
// observation{model, "default_policy/observation"},
// action{model, "default_policy/cond/Merge"}
// SAC stable-baselines
// observation{model, "input/input/Ob"},
// action{model, "model/Tanh"}
{
    // based on input, should choose mode (what will be listening for / using)

    // Rl interface
    // rl_actions_sub = nm.subscribe("/laikago_gazebo/rl_leg_cmds", 1, &PybulletInterface::setLegCommands, this);
    // rl_obs_pub = nm.advertise<laikago_msgs::RLObs>("/laikago_gazebo/rl_obs", 1);
    // rl_act_sub = nm.subscribe("/laikago_gazebo/rl_act", 1, &PybulletInterface::processAndSetLegCommandsFromRL, this);
    // nm.advertise<laikago_msgs::LowState>("/aliengo_gazebo/lowState/state", 1);

    // Tensorflow interface
    // tf_service = nm.advertiseService("/laikago_gazebo/tf_comm", &PybulletInterface::queryTensorflowModelService, this);

    // Gazebo
    gazebo_pause_srv = nm.serviceClient<std_srvs::Empty>("/gazebo/pause_physics");
    gazebo_unpause_srv = nm.serviceClient<std_srvs::Empty>("/gazebo/unpause_physics");

    // python_available_sub = nm.subscribe("/laikago_gazebo/python_available", 1, &PybulletInterface::turnOn, this);
    // std::cout << "before client" << std::endl;
    // rl_client = nm.serviceClient<laikago_msgs::RLService>("/laikago_gazebo/rl_comm");
    // std::cout << "after client" << std::endl;

    setupTensorflow();
    loadRLSpaceParams();

    for (int i = 0; i < 10; i++)
    {
        sensory_obs.push_back(vector<float>(28));
    }
    obs_rms_mean.resize(10 * 28);
    obs_rms_var.resize(10 * 28);
    obs_high.resize(10 * 28);

    // mytime.start();
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
void JumpingObj::getFullState(std::vector<double> &full_state)
{
    // data
    auto &seResult = _controlData->_stateEstimator->getResult();
    // idx
    int idx = 0;
    // body pos
    for (int i = 0; i < 3; i++)
    {
        full_state[idx + i] = seResult.position[i];
    }
    idx += 3;
    // body rpy
    for (int i = 0; i < 3; i++)
    {
        full_state[idx + i] = seResult.rpy[i];
    }
    idx += 3;
    // body frame linear velocity
    for (int i = 0; i < 3; i++)
    {
        full_state[idx + i] = seResult.vBody[i];
    }
    idx += 3;
    // body frame angular velocity
    for (int i = 0; i < 3; i++)
    {
        full_state[idx + i] = seResult.omegaBody[i];
    }
    idx += 3;
    for (int i = 0; i < 12; i++)
    {
        full_state[idx + i] = lowState.motorState[i].position;
        full_state[idx + i + 12] = lowState.motorState[i].velocity;
        full_state[idx + i + 24] = lowState.motorState[i].torque;
    }
    idx += 36;
    // foot positions and velocities
    for (int leg = 0; leg < 4; leg++)
    {
        // foot pos
        for (int j = 0; j < 3; j++)
        {
            full_state[idx + j] = _controlData->_legController->data[leg].p[j];
        }
        idx += 3;
        // foot vel
        for (int j = 0; j < 3; j++)
        {
            full_state[idx + j] = _controlData->_legController->data[leg].v[j];
        }
        idx += 3;
    }
    // contact bools
    // std::cout << "==" << std::endl;
    for (int i = 0; i < 4; i++)
    {
        // if (lowState.footForce[i] > 0){
        //     full_state[idx+i] = 1; // positive force, so in contact
        // }
        // else{
        //     full_state[idx+i] = 0; // in air
        // }
        full_state[idx + i] = lowState.footForce[i];
        // std::cout << _controlData->_legController->commands[i].feedforwardForce << std::endl;
    }
    idx += 4;

    // for (int i=0; i<12; i++){
    //     full_state[idx+i] = act_arr[i];
    // }

    full_state[idx] = act_arr[0];
    full_state[idx + 1] = 0;
    full_state[idx + 2] = act_arr[1];
    full_state[idx + 3] = act_arr[0];
    full_state[idx + 4] = 0;
    full_state[idx + 5] = act_arr[1];
    full_state[idx + 6] = act_arr[2];
    full_state[idx + 7] = 0;
    full_state[idx + 8] = act_arr[3];
    full_state[idx + 9] = act_arr[2];
    full_state[idx + 10] = 0;
    full_state[idx + 11] = act_arr[3];

    idx += 12;

    for (int i = 0; i < 4; i++)
    {
        full_state[idx + i] = _controlData->_legController->commands[i].feedforwardForce[2];
    }

    idx += 4;

    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
        {
            full_state[idx + i * 3 + j] = _controlData->_legController->commands[i].tau[j]; // torque send to motor
        }
    }
}

void JumpingObj::standing()
{

    std::cout << "QP activated" << std::endl;
    double FR = 0;
    double FL = 0;
    double RR = 0;
    double RL = 0; // contact state
    double Kp_l = 5;
    double Kd_l = 1;

    // set desired joint position and velocity
    _controlData->_legController->commands[0].qDes << 0, 1.2566 - 0.5, -2.355; // front right leg
    _controlData->_legController->commands[1].qDes << _controlData->_legController->commands[0].qDes;
    _controlData->_legController->commands[2].qDes << 0, 1.2566 - 0.5, -2.355; // rear right leg
    _controlData->_legController->commands[3].qDes << _controlData->_legController->commands[2].qDes;

    _controlData->_legController->commands[0].qdDes << 0, 0, 0; // front right leg
    _controlData->_legController->commands[1].qdDes << _controlData->_legController->commands[0].qdDes;
    _controlData->_legController->commands[2].qdDes << 0, 0, 0; // rear right leg
    _controlData->_legController->commands[3].qdDes << _controlData->_legController->commands[2].qdDes;

    for (int i = 0; i < 3; i++)
    {
        p_des[i] = 0;
        v_des[i] = 0;
        omegaDes[i] = 0;
        rpy[i] = 0;
    }

    p_des[2] = 0.15; // standup height for A1

    // Get contact state
    FR = lowState.footForce[0];
    FL = lowState.footForce[1];
    RR = lowState.footForce[2];
    RL = lowState.footForce[3];

    std::cout << "actual_contactState: " << FR << "," << FL << "," << RR << "," << RL << std::endl;

    if (FR > 0)
    {
        FR = 1;
        _controlData->_legController->commands[0].kpJoint << Kp_l, 0, 0,
            0, Kp_l, 0,
            0, 0, Kp_l;
        _controlData->_legController->commands[0].kdJoint << Kd_l, 0, 0,
            0, Kd_l, 0,
            0, 0, Kd_l;
    }
    else
    {
        FR = 0;
    }

    if (FL > 0)
    {
        FL = 1;
        _controlData->_legController->commands[1].kpJoint << Kp_l, 0, 0,
            0, Kp_l, 0,
            0, 0, Kp_l;
        _controlData->_legController->commands[1].kdJoint << Kd_l, 0, 0,
            0, Kd_l, 0,
            0, 0, Kd_l;
    }
    else
    {
        FL = 0;
    }

    if (RR > 0)
    {
        RR = 1;
        _controlData->_legController->commands[2].kpJoint << Kp_l, 0, 0,
            0, Kp_l, 0,
            0, 0, Kp_l;
        _controlData->_legController->commands[2].kdJoint << Kd_l, 0, 0,
            0, Kd_l, 0,
            0, 0, Kd_l;
    }
    else
    {
        RR = 0;
    }

    if (RL > 0)
    {
        RL = 1;
        _controlData->_legController->commands[3].kpJoint << Kp_l, 0, 0,
            0, Kp_l, 0,
            0, 0, Kp_l;
        _controlData->_legController->commands[3].kdJoint << Kd_l, 0, 0,
            0, Kd_l, 0,
            0, 0, Kd_l;
    }
    else
    {
        RL = 0;
    }

    // double contactStateScheduled[4]={FR,FL,RR,RL};
    double contactStateScheduled[4] = {1, 1, 1, 1};
    // std::cout << "set_contactState: " << contactStateScheduled[0] << "," << contactStateScheduled[1]<<"," << contactStateScheduled[2]<< "," << contactStateScheduled[3]<< std::endl;
    runQP = true;

    if (runQP)
    {
        //    _data->_stateEstimator->setContactPhase(contactphase);
        auto &seResult = _controlData->_stateEstimator->getResult();

        for (int i = 0; i < 4; i++)
        {
            se_xfb[i] = _controlData->_stateEstimator->getResult().orientation(i);
        }

        for (int i = 0; i < 3; i++)
        {
            p_act[i] = seResult.position(i);
            v_act[i] = seResult.vBody(i);

            se_xfb[4 + i] = seResult.position(i);
            se_xfb[7 + i] = seResult.omegaBody(i);
            se_xfb[10 + i] = seResult.vWorld(i);

            // Set the translational and orientation gain

            kpCOM[i] = 30;  //_data->controlParameters->kpCOM(i);
            kdCOM[i] = 10;  //_data->controlParameters->kdCOM(i);
            kpBase[i] = 80; //_data->controlParameters->kpBase(i);
            kdBase[i] = 20; //  _data->controlParameters->kdBase(i);
        }

        kpCOM[2] = 50;
        kdCOM[2] = 20;
        kpBase[0] = 300;
        // kdBase[0] = 5;
        kpBase[1] = 200;

        // Vec3<double> pFeetVec;
        Vec3<double> pFeetVecCOM;

        // Get the foot locations relative to COM
        for (int leg = 0; leg < 4; leg++)
        {
            // computeLegJacobianAndPosition(&_data->_quadruped, _data->_legController->data[leg].q,
            //                           (Mat3<double>*)nullptr, &pFeetVec, leg);
            // pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
            //(_data->_quadruped->getHipLocation(leg) + pFeetVec);

            pFeetVecCOM = seResult.rBody.transpose() *
                          (_controlData->_quadruped->getHipLocation(leg) + _controlData->_legController->data[leg].p);

            pFeet[leg * 3] = pFeetVecCOM[0];
            pFeet[leg * 3 + 1] = pFeetVecCOM[1];
            pFeet[leg * 3 + 2] = pFeetVecCOM[2];
            // std::cout << "pFeet" << leg << std::endl;
        }

        p_des[0] = p_act[0] + (pFeet[0] + pFeet[3] + pFeet[6] + pFeet[9]) / 4.0;
        p_des[1] = p_act[1] + (pFeet[1] + pFeet[4] + pFeet[7] + pFeet[10]) / 4.0;
        //  myfile << "\n";
        // std::cout << j << std::endl;
        // std::cout << "run QP" << std::endl;
        balanceController.set_alpha_control(0.01);
        balanceController.set_friction(0.2);
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
        // balanceController.get_b_matrix(b_control);
        // b_des << b_control[2] << "\n";

        // Publish the results over ROS
        // balanceController.publish_data_lcm();

        // Copy the results to the feed forward forces

        //_data->_stateEstimator->run();

        for (int leg = 0; leg < 4; leg++)
        {
            footFeedForwardForces.col(leg) << fOpt[leg * 3], fOpt[leg * 3 + 1],
                fOpt[leg * 3 + 2]; // force in world frame, need to convert to body frame

            _controlData->_legController->commands[leg].feedforwardForce = footFeedForwardForces.col(leg);
            //_data->_stateEstimator->getResult().rBody.transpose() * footFeedForwardForces.col(leg);
            // QP << _data->_legController->commands[leg].feedforwardForce[2] << " ";
        }

        // }

        computeTorquesandSend_standing();

        // cout<<"generated QP forces" << std::endl;
    }
}

void JumpingObj::computeTorquesandSend_standing()
{

    for (int i = 0; i < 4; i++)
    {
        // computeLegJacobianAndPosition(_quadruped, data[i].q,&(data[i].J),&(data[i].p),i);
        //  tauFF
        // commands[i].tau = Vec3<double>::Zero();
        Vec3<double> legTorque;
        // std::cout << "legTorque" << legTorque << std::endl;
        // forceFF

        Vec3<double> footForce = _controlData->_legController->commands[i].feedforwardForce;

        // footForce +=
        //     commands[i].kpCartesian * (commands[i].pDes - data[i].p);

        // footForce +=
        //     commands[i].kdCartesian * (commands[i].vDes - data[i].v);
        // std::cout << "leg: " << i << std::endl;
        // std::cout << footForce << std::endl;
        // torque
        legTorque += _controlData->_legController->data[i].J.transpose() * footForce;
        // std::cout << data[i].J << std::endl;
        _controlData->_legController->commands[i].tau = legTorque;
        for (int j = 0; j < 3; j++)
        {
            lowCmd.motorCmd[i * 3 + j].position = _controlData->_legController->commands[i].qDes[j];
            lowCmd.motorCmd[i * 3 + j].velocity = _controlData->_legController->commands[i].qdDes[j];
            lowCmd.motorCmd[i * 3 + j].torque = _controlData->_legController->commands[i].tau[j]; // legTorque[j];
            lowCmd.motorCmd[i * 3 + j].positionStiffness = _controlData->_legController->commands[i].kpJoint(j, j);
            lowCmd.motorCmd[i * 3 + j].velocityStiffness = _controlData->_legController->commands[i].kdJoint(j, j);
            // std::cout<<"pos_des:" << lowCmd.motorCmd[i*3+j].position << std::endl;
            // std::cout<<"vel_des:" << lowCmd.motorCmd[i*3+j].velocity << std::endl;
            // std::cout<< "Kp:" <<lowCmd.motorCmd[i*3+j].positionStiffness << std::endl;
            // std::cout<< "Kd:" <<lowCmd.motorCmd[i*3+j].velocityStiffness << std::endl;
        }
        // commands[i].tau << 0, 0, 0;
        legTorque << 0, 0, 0;
    }
    sendServoCmd();
    std::cout << "cmd sent" << std::endl;
}




void JumpingObj::landing(int counter)
{

    std::cout << "QP activated" << std::endl;
    double FR = 0;
    double FL = 0;
    double RR = 0;
    double RL = 0; // contact state
    // double Kp = 100;
    // double Kd = 2;
    double Kp_l = 2;
    double Kd_l = 1;

    // set desired joint position and velocity
    _controlData->_legController->commands[0].qDes << 0, 1.2566 - 0.8*1, -2.355; // front right leg
    _controlData->_legController->commands[1].qDes << _controlData->_legController->commands[0].qDes;
    // _controlData->_legController->commands[2].qDes << 0, 1.2566 - 0.8*0, -2.355; // rear right leg
    // _controlData->_legController->commands[3].qDes << _controlData->_legController->commands[2].qDes;

    _controlData->_legController->commands[0].qdDes << 0, 0, 0; // front right leg
    _controlData->_legController->commands[1].qdDes << _controlData->_legController->commands[0].qdDes;
    _controlData->_legController->commands[2].qdDes << 0, 0, 0; // rear right leg
    _controlData->_legController->commands[3].qdDes << _controlData->_legController->commands[2].qdDes;

    // if (counter >= 1000 && counter <= 1100)
    // {
    //     for (int i=0 ; i<4; i++){
    //         _controlData->_legController->commands[i].kpJoint << Kp, 0, 0,
    //                                                              0, Kp, 0,
    //                                                              0, 0, Kp;
    //         _controlData->_legController->commands[i].kdJoint << Kd, 0, 0,
    //                                                              0, Kd, 0,
    //                                                              0, 0, Kd;
    //     }
    // }

    for (int i = 0; i < 3; i++)
    {
        p_des[i] = 0;
        v_des[i] = 0;
        omegaDes[i] = 0;
        rpy[i] = 0;
    }

    p_des[2] = 0.2+0.1526; // standup height for A1

    // Get contact state
    FR = lowState.footForce[0];
    FL = lowState.footForce[1];
    RR = lowState.footForce[2];
    RL = lowState.footForce[3];

    std::cout << "actual_contactState: " << FR << "," << FL << "," << RR << "," << RL << std::endl;



    if (FR > 0)
    {
        FR = 1;
        _controlData->_legController->commands[0].kpJoint << Kp_l, 0, 0,
            0, Kp_l, 0,
            0, 0, Kp_l;
        _controlData->_legController->commands[0].kdJoint << Kd_l, 0, 0,
            0, Kd_l, 0,
            0, 0, Kd_l;
    }
    else
    {
        FR = 0;
    }

    if (FL > 0)
    {
        FL = 1;
        _controlData->_legController->commands[1].kpJoint << Kp_l, 0, 0,
            0, Kp_l, 0,
            0, 0, Kp_l;
        _controlData->_legController->commands[1].kdJoint << Kd_l, 0, 0,
            0, Kd_l, 0,
            0, 0, Kd_l;
    }
    else
    {
        FL = 0;
    }

    if (RR > 0)
    {
        RR = 1;
        _controlData->_legController->commands[2].kpJoint << Kp_l, 0, 0,
            0, Kp_l, 0,
            0, 0, Kp_l;
        _controlData->_legController->commands[2].kdJoint << Kd_l, 0, 0,
            0, Kd_l, 0,
            0, 0, Kd_l;
    }
    else
    {
        RR = 0;
    }

    if (RL > 0)
    {
        RL = 1;
        _controlData->_legController->commands[3].kpJoint << Kp_l, 0, 0,
            0, Kp_l, 0,
            0, 0, Kp_l;
        _controlData->_legController->commands[3].kdJoint << Kd_l, 0, 0,
            0, Kd_l, 0,
            0, 0, Kd_l;
    }
    else
    {
        RL = 0;
    }

    // double contactStateScheduled[4]={FR,FL,RR,RL};
    double contactStateScheduled[4] = {1, 1, 1, 1};
    // std::cout << "set_contactState: " << contactStateScheduled[0] << "," << contactStateScheduled[1]<<"," << contactStateScheduled[2]<< "," << contactStateScheduled[3]<< std::endl;
    
    if (counter > 1000){
        runQP = true;

    }


    if (runQP)
    {
        //    _data->_stateEstimator->setContactPhase(contactphase);
        auto &seResult = _controlData->_stateEstimator->getResult();

        for (int i = 0; i < 4; i++)
        {
            se_xfb[i] = _controlData->_stateEstimator->getResult().orientation(i);
        }

        for (int i = 0; i < 3; i++)
        {
            p_act[i] = seResult.position(i);
            v_act[i] = seResult.vBody(i);

            se_xfb[4 + i] = seResult.position(i);
            se_xfb[7 + i] = seResult.omegaBody(i);
            se_xfb[10 + i] = seResult.vWorld(i);

            // Set the translational and orientation gain

            kpCOM[i] = 30;  //_data->controlParameters->kpCOM(i);
            kdCOM[i] = 10;  //_data->controlParameters->kdCOM(i);
            kpBase[i] = 80; //_data->controlParameters->kpBase(i);
            kdBase[i] = 20; //  _data->controlParameters->kdBase(i);
        }

        kpCOM[2] = 50;
        kdCOM[2] = 20;
        kpBase[0] = 300;
        // kdBase[0] = 5;
        kpBase[1] = 150;

        // Vec3<double> pFeetVec;
        Vec3<double> pFeetVecCOM;

        // Get the foot locations relative to COM
        for (int leg = 0; leg < 4; leg++)
        {
            // computeLegJacobianAndPosition(&_data->_quadruped, _data->_legController->data[leg].q,
            //                           (Mat3<double>*)nullptr, &pFeetVec, leg);
            // pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
            //(_data->_quadruped->getHipLocation(leg) + pFeetVec);

            pFeetVecCOM = seResult.rBody.transpose() *
                          (_controlData->_quadruped->getHipLocation(leg) + _controlData->_legController->data[leg].p);

            pFeet[leg * 3] = pFeetVecCOM[0];
            pFeet[leg * 3 + 1] = pFeetVecCOM[1];
            pFeet[leg * 3 + 2] = pFeetVecCOM[2];
            // std::cout << "pFeet" << leg << std::endl;
        }

        p_des[0] = p_act[0] + (pFeet[0] + pFeet[3] + pFeet[6] + pFeet[9]) / 4.0;
        p_des[1] = p_act[1] + (pFeet[1] + pFeet[4] + pFeet[7] + pFeet[10]) / 4.0;
        //  myfile << "\n";
        // std::cout << j << std::endl;
        // std::cout << "run QP" << std::endl;
        balanceController.set_alpha_control(0.01);
        balanceController.set_friction(0.2);
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
        // balanceController.get_b_matrix(b_control);
        // b_des << b_control[2] << "\n";

        // Publish the results over ROS
        // balanceController.publish_data_lcm();

        // Copy the results to the feed forward forces

        //_data->_stateEstimator->run();

        for (int leg = 0; leg < 4; leg++)
        {
            footFeedForwardForces.col(leg) << fOpt[leg * 3], fOpt[leg * 3 + 1],
                fOpt[leg * 3 + 2]; // force in world frame, need to convert to body frame

            _controlData->_legController->commands[leg].feedforwardForce = footFeedForwardForces.col(leg);
            //_data->_stateEstimator->getResult().rBody.transpose() * footFeedForwardForces.col(leg);
            // QP << _data->_legController->commands[leg].feedforwardForce[2] << " ";
        }

        // }

        computeTorquesandSend_landing();

        // cout<<"generated QP forces" << std::endl;
    }
}

void JumpingObj::computeTorquesandSend_landing()
{

    for (int i = 0; i < 4; i++)
    {
        // computeLegJacobianAndPosition(_quadruped, data[i].q,&(data[i].J),&(data[i].p),i);
        //  tauFF
        // commands[i].tau = Vec3<double>::Zero();
        Vec3<double> legTorque;
        // std::cout << "legTorque" << legTorque << std::endl;
        // forceFF

        Vec3<double> footForce = _controlData->_legController->commands[i].feedforwardForce;

        // footForce +=
        //     commands[i].kpCartesian * (commands[i].pDes - data[i].p);

        // footForce +=
        //     commands[i].kdCartesian * (commands[i].vDes - data[i].v);
        // std::cout << "leg: " << i << std::endl;
        // std::cout << footForce << std::endl;
        // torque
        legTorque += _controlData->_legController->data[i].J.transpose() * footForce;
        // std::cout << data[i].J << std::endl;
        _controlData->_legController->commands[i].tau = legTorque;
        for (int j = 0; j < 3; j++)
        {
            lowCmd.motorCmd[i * 3 + j].position = _controlData->_legController->commands[i].qDes[j];
            lowCmd.motorCmd[i * 3 + j].velocity = _controlData->_legController->commands[i].qdDes[j];
            lowCmd.motorCmd[i * 3 + j].torque = _controlData->_legController->commands[i].tau[j]; // legTorque[j];
            lowCmd.motorCmd[i * 3 + j].positionStiffness = _controlData->_legController->commands[i].kpJoint(j, j);
            lowCmd.motorCmd[i * 3 + j].velocityStiffness = _controlData->_legController->commands[i].kdJoint(j, j);
            // std::cout<<"pos_des:" << lowCmd.motorCmd[i*3+j].position << std::endl;
            // std::cout<<"vel_des:" << lowCmd.motorCmd[i*3+j].velocity << std::endl;
            // std::cout<< "Kp:" <<lowCmd.motorCmd[i*3+j].positionStiffness << std::endl;
            // std::cout<< "Kd:" <<lowCmd.motorCmd[i*3+j].velocityStiffness << std::endl;
        }
        // commands[i].tau << 0, 0, 0;
        legTorque << 0, 0, 0;
    }
    sendServoCmd();
    std::cout << "cmd sent" << std::endl;
}

/****************************************************************************************************************************
*********************************************** RL Pre and post processing **************************************************
*****************************************************************************************************************************/

/**
 * Format observation to match pybullet format.
 *  TODO: add flags to easily switch between observation spaces
 */

void JumpingObj::get2DObservation_hist(int Iter, std::vector<double> init_final, std::vector<double> contactState) // Remove torque+ Add initial final config

{
    vector<float> curr_obs(27);
    auto &seResult = _controlData->_stateEstimator->getResult();

    // add initial CoM position and perturbation to observation
    curr_obs[0] = init_final[0]; // x
    curr_obs[1] = init_final[2]; // z
    curr_obs[2] = init_final[4]; // pitch
    int idx = 3;

    for (int i = 0; i < 2; i++)
        curr_obs[i + idx] = lowState.motorState[i + 1].position;
    idx += 2;
    for (int i = 0; i < 2; i++)
        curr_obs[i + idx] = lowState.motorState[i + 7].position;
    idx += 2;
    for (int i = 0; i < 2; i++)
        curr_obs[i + idx] = lowState.motorState[i + 1].velocity;
    idx += 2;
    for (int i = 0; i < 2; i++)
        curr_obs[i + idx] = lowState.motorState[i + 7].velocity;
    idx += 2;

    // orientation
    curr_obs[idx] = seResult.rpy[1];

    idx += 1;

    cout << "orientation: " << seResult.orientation[1] << ", " << seResult.orientation[2] << "" << seResult.orientation[3] << seResult.orientation[0] << endl;
    cout << "rpy:" << seResult.rpy[0] << "," << seResult.rpy[1] << "," << seResult.rpy[2] << endl;

    // x,z position
    curr_obs[idx] = seResult.position[0];
    curr_obs[idx + 1] = seResult.position[2];

    idx += 2;

    // world frame velocity (want world frame for pybullet)
    Vec3<double> wFrameVel = seResult.rBody.transpose() * seResult.vBody;
    curr_obs[idx] = wFrameVel[0];
    curr_obs[idx + 1] = wFrameVel[2];
    idx += 2;

    // angular velocity
    curr_obs[idx] = seResult.omegaBody[1];
    idx += 1;
    // foot pos /vel [TODO: add flag]

    // foot pos
    curr_obs[idx] = _controlData->_legController->data[0].p[0];     // front pfoot_x
    curr_obs[idx + 1] = _controlData->_legController->data[0].p[2]; // front pfoot_z
    curr_obs[idx + 2] = _controlData->_legController->data[2].p[0]; // rear pfoot_x
    curr_obs[idx + 3] = _controlData->_legController->data[2].p[2]; // rear pfoot_z

    idx += 4;

    // foot vel
    curr_obs[idx] = _controlData->_legController->data[0].v[0];     // front pfoot_x
    curr_obs[idx + 1] = _controlData->_legController->data[0].v[2]; // front pfoot_z
    curr_obs[idx + 2] = _controlData->_legController->data[2].v[0]; // rear pfoot_x
    curr_obs[idx + 3] = _controlData->_legController->data[2].v[2]; // rear pfoot_z

    idx += 4;

    // get contact bools from calculation

    curr_obs[idx] = contactState[0];
    curr_obs[idx + 1] = contactState[2];

    // int u = min(Iter+20*1, full_opt_N);
    // cout << "u1:" << u << std::endl;
    // for(int i=0; i<72;i++){

    //     curr_obs.push_back(x_opt[u-1][i]);
    // }

    // u = min(Iter+20*5, full_opt_N);
    // cout << "u3:" << u << std::endl;
    // for(int i=0; i<72;i++){
    //     curr_obs.push_back(x_opt[u-1][i]);
    // }

    // u = min(Iter+20*17, full_opt_N);
    // cout << "u5:" << u << std::endl;
    // for(int i=0; i<72;i++){
    //     curr_obs.push_back(x_opt[u-1][i]);
    // }

    // for (int i = 0; i< 28; i++){
    //     cout << "curr_obs:" << i << " " << curr_obs[i] << endl;
    // }

    sensory_obs.pop_front();
    sensory_obs.push_back(curr_obs);

    // // To check the last element to see if it is curr_obs
    // auto ptr = sensory_obs.begin();
    // std::advance(ptr, 29);
    // std::vector<float> obs = *ptr;
    // for (int i =0; i<77; i++){
    //     cout << obs.at(i) << endl;
    // }

    // cout <<"pass here:" << endl;

    int c = 0;
    // flatten sensory obs

    for (auto i = sensory_obs.begin(); i != sensory_obs.end(); i++)
    {
        for (auto j = i->begin(); j != i->end(); j++)
        {
            // myfile << *j << ",";
            // cout << "j: " << *j << ", ";
            rl_obs.data[c++] = *j;
        }
    }

    // To check 77 last elements of rl_obs to see it is curr_obs
    // for (int i =2310-1-77; i<2310; i++){
    //     cout << rl_obs.data[i] << endl;
    // }

    //    rl_obs.data[c++] = 1;
    cout << "counter value after pushing all obs into flat list: " << c << endl;
}

void JumpingObj::getObservation_hist_v3(int Iter, std::vector<double> init_final, std::vector<double> contactState) // Remove torque+ Add initial final config

{
    vector<float> curr_obs(77);
    auto &seResult = _controlData->_stateEstimator->getResult();

    // use initial and desired configuration to observation
    for (int i = 0; i < 12; i++)
    {
        curr_obs[i] = init_final[i];
        // cout<< "init_final:" << curr_obs[i] << std::endl;
    }

    int idx = 12;

    // rl_obs will have the observation
    // joints: lowState.motorState[i].{position, velocity, torque}
    for (int i = 0; i < 12; i++)
        curr_obs[i + idx] = lowState.motorState[i].position;
    idx += 12;
    for (int i = 0; i < 12; i++)
        curr_obs[i + idx] = lowState.motorState[i].velocity;
    // for(int i = 0; i < 12; i++) curr_obs[i+24]=lowState.motorState[i].torque;

    idx += 12;
    // orientation
    curr_obs[idx] = seResult.orientation[1];
    curr_obs[idx + 1] = seResult.orientation[2];
    curr_obs[idx + 2] = seResult.orientation[3];
    curr_obs[idx + 3] = seResult.orientation[0];

    idx += 4;

    cout << "orientation: " << seResult.orientation[1] << ", " << seResult.orientation[2] << "" << seResult.orientation[3] << seResult.orientation[0] << endl;
    cout << "rpy:" << seResult.rpy[0] << "," << seResult.rpy[1] << "," << seResult.rpy[2] << endl;
    // cout << "position" << lowState.motorState[2].position << endl;

    // z position
    for (int i = 0; i < 3; i++)
        curr_obs[idx + i] = seResult.position[i];

    idx += 3;

    // world frame velocity (want world frame for pybullet)
    Vec3<double> wFrameVel = seResult.rBody.transpose() * seResult.vBody;
    for (int i = 0; i < 3; i++)
        curr_obs[idx + i] = wFrameVel[i];
    idx += 3;
    // angular velocity
    for (int i = 0; i < 3; i++)
        curr_obs[idx + i] = seResult.omegaBody[i];
    idx += 3;
    // foot pos /vel [TODO: add flag]

    for (int leg = 0; leg < 4; leg++)
    {
        // foot pos
        for (int j = 0; j < 3; j++)
        {
            curr_obs[idx + j] = _controlData->_legController->data[leg].p[j];
        }
        idx += 3;
    }

    for (int leg = 0; leg < 4; leg++)
    {
        // foot vel
        for (int j = 0; j < 3; j++)
        {
            curr_obs[idx + j] = _controlData->_legController->data[leg].v[j];
        }
        idx += 3;
    }

    // contact bools
    // for (int i = 0; i < 4; i++){
    //     if (lowState.footForce[i] > 0){
    //         //rl_obs.data[idx+i] = 1; // positive force, so in contact
    //         curr_obs[idx+i] = 1;
    //     }
    //     else{
    //         //rl_obs.data[idx+i] = 0; // in air
    //         curr_obs[idx+i] = 0;
    //     }
    // }

    // get contact bools from calculation
    for (int i = 0; i < 4; i++)
    {
        curr_obs[idx + i] = contactState[i];
    }

    // // add traj opt last state (goal)
    // for (int j=0; j<5; j++){
    //         for (int i=0; i<72; i++){
    //             curr_obs.push_back(x_opt[full_opt_N-1][i]); // original
    //         }
    // }

    int u = min(Iter + 20 * 1, full_opt_N);
    cout << "u1:" << u << std::endl;
    for (int i = 0; i < 72; i++)
    {

        curr_obs.push_back(x_opt[u - 1][i]);
    }

    u = min(Iter + 20 * 5, full_opt_N);
    cout << "u3:" << u << std::endl;
    for (int i = 0; i < 72; i++)
    {
        curr_obs.push_back(x_opt[u - 1][i]);
    }

    u = min(Iter + 20 * 17, full_opt_N);
    cout << "u5:" << u << std::endl;
    for (int i = 0; i < 72; i++)
    {
        curr_obs.push_back(x_opt[u - 1][i]);
    }

    // for (int i = 0; i< 77; i++){
    //     cout << "curr_obs:" << i << " " << curr_obs[i] << endl;
    // }

    sensory_obs.pop_front();
    sensory_obs.push_back(curr_obs);

    // // To check the last element to see if it is curr_obs
    // auto ptr = sensory_obs.begin();
    // std::advance(ptr, 29);
    // std::vector<float> obs = *ptr;
    // for (int i =0; i<77; i++){
    //     cout << obs.at(i) << endl;
    // }

    // cout <<"pass here:" << endl;

    int c = 0;
    // flatten sensory obs

    for (auto i = sensory_obs.begin(); i != sensory_obs.end(); i++)
    {
        for (auto j = i->begin(); j != i->end(); j++)
        {
            // myfile << *j << ",";
            // cout << "j: " << *j << ", ";
            rl_obs.data[c++] = *j;
        }
    }

    // To check 77 last elements of rl_obs to see it is curr_obs
    // for (int i =2310-1-77; i<2310; i++){
    //     cout << rl_obs.data[i] << endl;
    // }

    //    rl_obs.data[c++] = 1;
    cout << "counter value after pushing all obs into flat list: " << c << endl;
}

void JumpingObj::getObservation()
{
    auto &seResult = _controlData->_stateEstimator->getResult();

    pyb_obs.clear();
    // rl_obs will have the observation
    // joints: lowState.motorState[i].{position, velocity, torque}
    for (int i = 0; i < 12; i++)
        pyb_obs.push_back(lowState.motorState[i].position);
    for (int i = 0; i < 12; i++)
        pyb_obs.push_back(lowState.motorState[i].velocity);
    for (int i = 0; i < 12; i++)
        pyb_obs.push_back(lowState.motorState[i].torque);

    // orientation
    pyb_obs.push_back(lowState.cheat.orientation[1]);
    pyb_obs.push_back(lowState.cheat.orientation[2]);
    pyb_obs.push_back(lowState.cheat.orientation[3]);
    pyb_obs.push_back(lowState.cheat.orientation[0]);
    // z position
    for (int i = 0; i < 3; i++)
        pyb_obs.push_back(lowState.cheat.position[i]);

    // world frame velocity (want world frame for pybullet)
    Vec3<double> wFrameVel = seResult.rBody.transpose() * seResult.vBody;
    for (int i = 0; i < 3; i++)
        pyb_obs.push_back(wFrameVel[i]);
    // angular velocity
    for (int i = 0; i < 3; i++)
        pyb_obs.push_back(seResult.omegaBody[i]);

    // foot pos /vel [TODO: add flag]
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
            pyb_obs.push_back(_controlData->_legController->data[i].p[j]);
        // for (int j=0; j<3; j++) pyb_obs.push_back(_controlData->_legController->data[i].v[j]);
    }
    for (int i = 0; i < 4; i++)
    {
        for (int j = 0; j < 3; j++)
            pyb_obs.push_back(_controlData->_legController->data[i].v[j]);
    }

    // add traj opt last state (goal)
    for (int i = 0; i < opt_state_space; i++)
    {
        pyb_obs.push_back(x_opt[full_opt_N - 1][i]); // original
        // pyb_obs.push_back(x_opt[full_opt_N-1-12][i]); // chuong modified to make pyb_obs has same size with obs-> but not good performance
    }

    // contact bools
    for (int i = 0; i < 4; i++)
    {
        // if (lowState.footForce[i] > 0){
        //     //rl_obs.data[idx+i] = 1; // positive force, so in contact
        //     pyb_obs.push_back(1);
        // }
        // else{
        //     //rl_obs.data[idx+i] = 0; // in air
        //     pyb_obs.push_back(0);
        // }

        // it's in contact at the beginning, for sure
        pyb_obs.push_back(1);
    }

    // check the observation - is it the same?
    // std::cout << "======================================================================" << std::endl;
    // std::cout << "checking obs..." << std::endl;
    // for (int i = 0; i < obs_len ; i++){
    //     std::cout << i << " " << rl_obs.data[i] << std::endl;
    // }
}
/**
 * Send raw obs to pybullet
 */
void JumpingObj::sendObservation()
{
    // construct obs
    getObservation();
    // publish
    rl_obs_pub.publish(rl_obs);
}

/*
 * Given traj index, set the default commands
 */
void JumpingObj::setLegControllerCommandsFromTrajIndex(int trajIdx)
{
    kpCartesian << 500, 0, 0,
        0, 500, 0,
        0, 0, 500; // 100
    kdCartesian << 10, 0, 0,
        0, 10, 0,
        0, 0, 10;

    kpJoint << 500, 0, 0,
        0, 500, 0,
        0, 0, 500; // 100
    kdJoint << 5, 0, 0,
        0, 5, 0,
        0, 0, 5;

    // kpJoint << 300, 0, 0,
    //             0, 300, 0,
    //             0, 0, 300; //100
    // kdJoint<<   3, 0, 0,
    //             0, 3, 0,
    //             0, 0, 3;

    auto des_state = x_opt[trajIdx];
    for (int leg = 0; leg < 4; leg++)
    {
        // set these from where? from opt, as pass in values to function?
        Vec3<double> qDes, qdDes, tau, pDes, vDes; //(des_state.begin()+JOINT_POS_INDEX,);
        for (int j = 0; j < 3; j++)
        {
            // qDes.push_back(des_state[JOINT_POS_INDEX+leg*3+j]);
            // qdDes.push_back(des_state[JOINT_VEL_INDEX+leg*3+j]);
            // tau.push_back(des_state[TORQUE_INDEX+leg*3+j]);
            // pDes.push_back(des_state[FOOT_POS_INDEX+leg*3+j]);
            // vDes.push_back(des_state[FOOT_VEL_INDEX+leg*3+j]);
            qDes[j] = des_state[JOINT_POS_INDEX + leg * 3 + j];
            qdDes[j] = des_state[JOINT_VEL_INDEX + leg * 3 + j];
            tau[j] = des_state[TORQUE_INDEX + leg * 3 + j];
            pDes[j] = des_state[FOOT_POS_INDEX + leg * 3 + j];
            vDes[j] = des_state[FOOT_VEL_INDEX + leg * 3 + j];
        }

        _controlData->_legController->commands[leg].qDes = qDes; // des_state[JOINT_POS_INDEX+leg*3:JOINT_POS_INDEX+leg*3+3];
        _controlData->_legController->commands[leg].qdDes = qdDes;
        _controlData->_legController->commands[leg].tau = tau;
        _controlData->_legController->commands[leg].pDes = pDes;
        _controlData->_legController->commands[leg].vDes = vDes;
        // gains
        _controlData->_legController->commands[leg].kpJoint = kpJoint;
        _controlData->_legController->commands[leg].kdJoint = kdJoint;
        _controlData->_legController->commands[leg].kpCartesian = kpCartesian;
        _controlData->_legController->commands[leg].kdCartesian = kdCartesian;
        // print
        // cout<<"pDes of leg"<< leg <<":"<< pDes << endl;
        // cout<< "-------------" << endl;
    }

    // // add RL contribution
    // // if (USE_RL && flight_idx < 800){
    // if (USE_RL && trajIdx < flight_idx){
    //     if(trajIdx == 0){
    //         rl_action_idx = 0;
    //     }
    //     else if(trajIdx % 100 == 0){
    //         rl_action_idx += 1;
    //     }
    // std::cout << "rl_action_idx " << rl_action_idx << std::endl;
    updateLegControllerCommandsFromRL();
    // }
}

void JumpingObj::setLegControllerCommandsFromTrajIndex_2D(int trajIdx)
{
    kpCartesian << 500, 0, 0,
        0, 500, 0,
        0, 0, 500; // 100
    kdCartesian << 10, 0, 0,
        0, 10, 0,
        0, 0, 10;

    // kpJoint << 500, 0, 0,
    //     0, 500, 0,
    //     0, 0, 500; // 100
    // kdJoint << 5, 0, 0,
    //     0, 5, 0,
    //     0, 0, 5;

    kpJoint << 300, 0, 0,
                0, 300, 0,
                0, 0, 300; //100
    kdJoint<<   3, 0, 0,
                0, 3, 0,
                0, 0, 3;

    auto des_state = x_opt[trajIdx];
    for (int leg = 0; leg < 4; leg++)
    {
        // set these from where? from opt, as pass in values to function?
        Vec3<double> qDes, qdDes, tau, pDes, vDes; //(des_state.begin()+JOINT_POS_INDEX,);
        for (int j = 0; j < 3; j++)
        {
            // qDes.push_back(des_state[JOINT_POS_INDEX+leg*3+j]);
            // qdDes.push_back(des_state[JOINT_VEL_INDEX+leg*3+j]);
            // tau.push_back(des_state[TORQUE_INDEX+leg*3+j]);
            // pDes.push_back(des_state[FOOT_POS_INDEX+leg*3+j]);
            // vDes.push_back(des_state[FOOT_VEL_INDEX+leg*3+j]);
            qDes[j] = des_state[JOINT_POS_INDEX + leg * 3 + j];
            qdDes[j] = des_state[JOINT_VEL_INDEX + leg * 3 + j];
            tau[j] = des_state[TORQUE_INDEX + leg * 3 + j];
            pDes[j] = des_state[FOOT_POS_INDEX + leg * 3 + j];
            vDes[j] = des_state[FOOT_VEL_INDEX + leg * 3 + j];
        }

        _controlData->_legController->commands[leg].qDes = qDes; // des_state[JOINT_POS_INDEX+leg*3:JOINT_POS_INDEX+leg*3+3];
        _controlData->_legController->commands[leg].qdDes = qdDes;
        _controlData->_legController->commands[leg].tau = tau;
        _controlData->_legController->commands[leg].pDes = pDes;
        _controlData->_legController->commands[leg].vDes = vDes;
        // gains
        _controlData->_legController->commands[leg].kpJoint = kpJoint;
        _controlData->_legController->commands[leg].kdJoint = kdJoint;
        _controlData->_legController->commands[leg].kpCartesian = kpCartesian;
        _controlData->_legController->commands[leg].kdCartesian = kdCartesian;
        // print
        // cout<<"pDes of leg"<< leg <<":"<< pDes << endl;
        // cout<< "-------------" << endl;
    }

    updateLegControllerCommandsFromRL_2D();
    // }
}

void JumpingObj::moveToTrajInitPos()
{
    std::cout << "reset A1 to starting pos..... " << std::endl;
    paramJumpInit();
    double pos[12];
    for (int i = 0; i < 12; i++)
    {
        pos[i] = x_opt[0][JOINT_POS_INDEX + i];
        // cout<<"pos" << i << pos[i]<<endl;
    }
    moveAllPosition(pos, 5000);
    std::cout << "... done " << std::endl;
}

/**
 * AFTER calling setLegControllerCommandsFromTrajIndex(),  add the RL contribution
 *  -can also not add, if just want to test opt
 */
void JumpingObj::updateLegControllerCommandsFromRL()
{

    for (int i = 0; i < 4; i++)
    {
        // updates Jacobian (not really needed here?)
        computeLegJacobianAndPosition(_controlData->_legController->_quadruped,
                                      _controlData->_legController->data[i].q,
                                      &(_controlData->_legController->data[i].J),
                                      &(_controlData->_legController->data[i].p), i);

        // this is the delta foot position learned with RL
        Vec3<double> delta_p(act_arr[i * 3],
                             act_arr[i * 3 + 1],
                             act_arr[i * 3 + 2]);

        // cout<<"delta_p:" << delta_p<< endl;

        // get contribution in joint space
        auto des_q = _controlData->_legController->commands[i].qDes;
        Mat3<double> J_qdes;
        Vec3<double> p_temp; // not going to be used
        // std::cout << "motor joint pos cmd: " << _controlData->_legController->commands[i].qDes << std::endl;

        computeLegJacobianAndPosition(_controlData->_legController->_quadruped,
                                      des_q,
                                      &J_qdes,
                                      &p_temp, i);

        // update desired q
        _controlData->_legController->commands[i].qDes += J_qdes.transpose() * delta_p;
        // update desired foot pos
        _controlData->_legController->commands[i].pDes += delta_p;
    }
    cout << "----------------" << endl;
}

void JumpingObj::updateLegControllerCommandsFromRL_2D()
{

    for (int i = 0; i < 4; i++)
    {
        // updates Jacobian (not really needed here?)
        computeLegJacobianAndPosition(_controlData->_legController->_quadruped,
                                      _controlData->_legController->data[i].q,
                                      &(_controlData->_legController->data[i].J),
                                      &(_controlData->_legController->data[i].p), i);

        // this is the delta foot position learned with RL
        Vec3<double> delta_p(0,
                             0,
                             0);

        if (i < 2)
        {
            delta_p[0] = act_arr[0];
            delta_p[2] = act_arr[1];
        }
        else
        {
            delta_p[0] = act_arr[2];
            delta_p[2] = act_arr[3];
        }

        cout << "delta_p:" << delta_p << endl;

        // get contribution in joint space
        auto des_q = _controlData->_legController->commands[i].qDes;
        Mat3<double> J_qdes;
        Vec3<double> p_temp; // not going to be used
        // std::cout << "motor joint pos cmd: " << _controlData->_legController->commands[i].qDes << std::endl;

        computeLegJacobianAndPosition(_controlData->_legController->_quadruped,
                                      des_q,
                                      &J_qdes,
                                      &p_temp, i);

        // std::cout << "delta_p:" << delta_p << std::endl;

        // update desired q
        _controlData->_legController->commands[i].qDes += J_qdes.transpose() * delta_p;
        // update desired foot pos
        _controlData->_legController->commands[i].pDes += delta_p;
    }
    cout << "----------------" << endl;
}

/**
 * Actually compute torques from commands, and send to robot
 */
void JumpingObj::computeFullTorquesAndSend()
{
    for (int i = 0; i < 4; i++)
    {

        Vec3<double> legTorque = -1.0 * _controlData->_legController->commands[i].tau;
        // std::cout << "commmand" << commands[i].tau << std::endl;
        //  forceFF

        Vec3<double> tau_ff = legTorque + _controlData->_legController->commands[i].kpJoint * (_controlData->_legController->commands[i].qDes - _controlData->_legController->data[i].q) + _controlData->_legController->commands[i].kdJoint * (_controlData->_legController->commands[i].qdDes - _controlData->_legController->data[i].qd);

        // Jacobian already updated!
        Vec3<double> tau_Cartesian(0, 0, 0);
        tau_Cartesian +=
            _controlData->_legController->commands[i].kpCartesian * (_controlData->_legController->commands[i].pDes - _controlData->_legController->data[i].p);

        tau_Cartesian +=
            _controlData->_legController->commands[i].kdCartesian * (_controlData->_legController->commands[i].vDes - _controlData->_legController->data[i].v);

        // torque
        tau_Cartesian = _controlData->_legController->data[i].J.transpose() * tau_Cartesian;

        // NOTE: this is changed to just be ffwed torque and Cartesian PD, the rest is in joint PD
        _controlData->_legController->commands[i].tau = legTorque + tau_Cartesian; // tau_ff + tau_Cartesian;
        // _controlData->_legController->commands[i].tau = legTorque + tau_Cartesian; //tau_ff + tau_Cartesian;

        // for (int j = 0; j<3; j++){
        //     lowCmd.motorCmd[i*3+j].torque = _controlData->_legController->commands[i].tau(j);

        //  //std::cout << "motor torque cmd: " << lowCmd.motorCmd[i*3+j].torque << std::endl;
        //  //std::cout << commands[i].tau(j) << std::endl;
        // }

        // update as in jump.cpp, with Cartesian PD added to torque
        for (int j = 0; j < 3; j++)
        {
            lowCmd.motorCmd[i * 3 + j].position = _controlData->_legController->commands[i].qDes[j];
            lowCmd.motorCmd[i * 3 + j].velocity = _controlData->_legController->commands[i].qdDes[j];
            lowCmd.motorCmd[i * 3 + j].torque = _controlData->_legController->commands[i].tau[j]; // legTorque[j];
            lowCmd.motorCmd[i * 3 + j].positionStiffness = _controlData->_legController->commands[i].kpJoint(j, j);
            lowCmd.motorCmd[i * 3 + j].velocityStiffness = _controlData->_legController->commands[i].kdJoint(j, j);
        }


        
    }
    sendServoCmd();
}

void JumpingObj::computeFullTorquesAndSend_constraints()
{

    double Kt = 0.118; // 4/34
    double torque_motor_max = 4;
    double max_ms = 1700 * 2 * 3.14 / 60;
    double min_ms = 940 * 2 * 3.14 / 60;
    double _gear_ratio = 8.5;

    double max_js = max_ms/ _gear_ratio;
    double min_js = min_ms/ _gear_ratio;

    double min_ts = 0.2 * _gear_ratio; // # min torque from plot
    double max_ts = torque_motor_max * _gear_ratio; // # max torque from plot

    // the parameters of green lines, converted to joint instead of motor
    double alpha_joint = (min_js - max_js)/(max_ts - min_ts); 
    double bj= min_js-alpha_joint * max_ts; 

    double _voltage_max = 21.5;
    double _current_max = 59.99;
    
    double _joint_vel_limit = 21;
    double _joint_torque_max = 33.5;
    double _R_motor = 25 * Kt * Kt;
    // double _R_motor = 0.638;
    double voltage[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // voltage for all joints
    double current[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // current for all joints
    double total_current = 0;
    double total_torque = 0;
    double total_power = 0 ;
    // cout << "max_js:" << max_js << std::endl;


    for (int i = 0; i < 4; i++)
    {

        Vec3<double> legTorque = -1.0 * _controlData->_legController->commands[i].tau;
        // std::cout << "commmand" << commands[i].tau << std::endl;
        //  forceFF

        Vec3<double> tau_ff = legTorque + _controlData->_legController->commands[i].kpJoint * (_controlData->_legController->commands[i].qDes - _controlData->_legController->data[i].q) + _controlData->_legController->commands[i].kdJoint * (_controlData->_legController->commands[i].qdDes - _controlData->_legController->data[i].qd);

        // Jacobian already updated!
        Vec3<double> tau_Cartesian(0, 0, 0);
        tau_Cartesian +=
            _controlData->_legController->commands[i].kpCartesian * (_controlData->_legController->commands[i].pDes - _controlData->_legController->data[i].p);

        tau_Cartesian +=
            _controlData->_legController->commands[i].kdCartesian * (_controlData->_legController->commands[i].vDes - _controlData->_legController->data[i].v);

        // torque
        tau_Cartesian = _controlData->_legController->data[i].J.transpose() * tau_Cartesian;

        // Vec3<double> tau_PD = _controlData->_legController->commands[i].kpJoint *
        //                           (_controlData->_legController->commands[i].qDes - _controlData->_legController->data[i].q) +
        //                       _controlData->_legController->commands[i].kdJoint *
        //                           (_controlData->_legController->commands[i].qdDes - _controlData->_legController->data[i].qd);

        // _controlData->_legController->commands[i].tau = legTorque + tau_Cartesian; //tau_ff + tau_Cartesian;

        _controlData->_legController->commands[i].tau = tau_ff + tau_Cartesian; // full torque

        // for (int j = 0; j<3; j++){
        //     lowCmd.motorCmd[i*3+j].torque = _controlData->_legController->commands[i].tau(j);

        //  //std::cout << "motor torque cmd: " << lowCmd.motorCmd[i*3+j].torque << std::endl;
        //  //std::cout << commands[i].tau(j) << std::endl;
        // }

    }
 

    // Limit joint velocity via torque adjustment

    // for (int i=0; i<4; i++){
    //     for (int j=0; j<3; j++){
    //         if(abs(_controlData->_legController->data[i].qd[j]) >= _joint_vel_limit*0.75)
    //         {
    //             _controlData->_legController->commands[i].tau[j] = (_controlData->_legController->data[i].qd[j]-bj)/alpha_joint;
    //         }
    //     }
    // }


    // NOTE: Limit total current / upper power, voltage, lower power

    // Limit total current consumed by all motors (Unitree specs)
    // for (int i=0; i<4; i++){
    //     for (int j = 0; j < 3; j++)
    //     {
    //         if (_controlData->_legController->commands[i].tau[j] >= _joint_torque_max)
    //         {
    //             _controlData->_legController->commands[i].tau[j] = _joint_torque_max;
    //         }

    //         if (_controlData->_legController->commands[i].tau[j] <= -_joint_torque_max)
    //         {
    //             _controlData->_legController->commands[i].tau[j] = -_joint_torque_max;
    //         }

    //         total_torque += _controlData->_legController->commands[i].tau[j]; // total torque
    //     }
    // }

    // // cout << "total torque:" << total_torque << std::endl;
    // total_current = total_torque/(_gear_ratio*Kt); // total current

    // // cout << "total current:" << total_current << std::endl;

    // if (total_current > _current_max || total_current < -1 * _current_max)
    // {
    //         double K_c = abs(_current_max / total_current);
    //         for (int i = 0; i < 4; i++)
    //         {
    //             for (int j = 0; j < 3; j++)
    //             {
    //                 _controlData->_legController->commands[i].tau[j] = _controlData->_legController->commands[i].tau[j] * K_c;
    //             }
    //         }
    // }

    // Compute voltage, limit voltage via limit torque

    for (int i =0; i<4; i++){

        for (int j = 0; j < 3; j++)
        {
            voltage[i * 3 + j] = _controlData->_legController->commands[i].tau(j) * _R_motor / (Kt * _gear_ratio) + _controlData->_legController->data[i].qd[j] * _gear_ratio * Kt;
            
            // cout << "voltage:" << voltage[i * 3 + j] << std::endl;
            if (voltage[i * 3 + j] > _voltage_max)
            {
                _controlData->_legController->commands[i].tau[j] = (_voltage_max - 1.0 * _controlData->_legController->data[i].qd[j] * _gear_ratio * Kt) * (Kt * _gear_ratio / _R_motor);
            }
            if (voltage[i * 3 + j] < -1.0 * _voltage_max)
            {
                _controlData->_legController->commands[i].tau[j] = (-1.0 * _voltage_max - 1.0 * _controlData->_legController->data[i].qd[j] * _gear_ratio * Kt) * (Kt * _gear_ratio / _R_motor);
            }
        }


    }

    // Limit upper power

    for (int i =0; i<4; i++){

        for (int j = 0; j < 3; j++)
        {
            voltage[i * 3 + j] = _controlData->_legController->commands[i].tau(j) * _R_motor / (Kt * _gear_ratio) + _controlData->_legController->data[i].qd[j] * _gear_ratio * Kt;
            current[i * 3 + j] = _controlData->_legController->commands[i].tau(j)/ (Kt * _gear_ratio);
            total_power += voltage[i * 3 +j] * current[i *3 +j];
        }
    }

    double A = 0; double B =0;

    if (total_power > _voltage_max * _current_max){
      

      for (int i =0 ; i<4; i++){

        for (int j=0; j<3; j++){
           A += _R_motor * pow( _controlData->_legController->commands[i].tau(j)/(_gear_ratio*Kt), 2);
           B += _controlData->_legController->data[i].qd(j)*_controlData->_legController->commands[i].tau(j);
        }
      }
      
      // We want A*k^2+B*k = P_max
      double k = (-B + sqrt(pow(B, 2)+ 4 * A *_voltage_max * _current_max))/(2*A);

      for (int i = 0; i < 4; i++){
            for (int j = 0; j < 3; j++)
            {
                _controlData->_legController->commands[i].tau[j] = _controlData->_legController->commands[i].tau[j] * k;
            }
      }

    }

    // // Limit lower power >=0, don't want recharge back to battery
    // if (B <0){
    //     double Kc = -B/A;
    //     for (int i = 0; i < 4; i++){
    //         for (int j = 0; j < 3; j++)
    //         {
    //             _controlData->_legController->commands[i].tau[j] = _controlData->_legController->commands[i].tau[j] * Kc;
    //         }
    //     }
    // }


    
    // // limit torque --> limit current run through motor (Unitree specs)

    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 3; j++)
        {
            if (_controlData->_legController->commands[i].tau[j] >= _joint_torque_max)
            {
                _controlData->_legController->commands[i].tau[j] = _joint_torque_max;
            }

            if (_controlData->_legController->commands[i].tau[j] <= -_joint_torque_max)
            {
                _controlData->_legController->commands[i].tau[j] = -_joint_torque_max;
            }
        }
    }


    for (int i = 0; i < 4; i++)
    {
        Vec3<double> tau_PD = _controlData->_legController->commands[i].kpJoint *
                            (_controlData->_legController->commands[i].qDes - _controlData->_legController->data[i].q) +
                              _controlData->_legController->commands[i].kdJoint *
                            (_controlData->_legController->commands[i].qdDes - _controlData->_legController->data[i].qd);
        // send torque to motor
        for (int j = 0; j < 3; j++)
        {
            lowCmd.motorCmd[i * 3 + j].position = _controlData->_legController->commands[i].qDes[j];
            lowCmd.motorCmd[i * 3 + j].velocity = _controlData->_legController->commands[i].qdDes[j];
            lowCmd.motorCmd[i * 3 + j].torque = _controlData->_legController->commands[i].tau[j] - 1 * tau_PD[j]; // legTorque[j];
            lowCmd.motorCmd[i * 3 + j].positionStiffness = _controlData->_legController->commands[i].kpJoint(j, j);
            lowCmd.motorCmd[i * 3 + j].velocityStiffness = _controlData->_legController->commands[i].kdJoint(j, j);
        }
    }

    // Send command to motor servo
    sendServoCmd();
}


void JumpingObj::computeFullTorquesAndSend_constraints_v2() // torque limits only
{
    double _joint_vel_limit = 21;
    double _joint_torque_max = 33.5;

    for (int i = 0; i < 4; i++)
    {

        Vec3<double> legTorque = -1.0 * _controlData->_legController->commands[i].tau;
        // std::cout << "commmand" << commands[i].tau << std::endl;
        //  forceFF

        Vec3<double> tau_ff = legTorque + _controlData->_legController->commands[i].kpJoint * (_controlData->_legController->commands[i].qDes - _controlData->_legController->data[i].q) + _controlData->_legController->commands[i].kdJoint * (_controlData->_legController->commands[i].qdDes - _controlData->_legController->data[i].qd);

        // Jacobian already updated!
        Vec3<double> tau_Cartesian(0, 0, 0);
        tau_Cartesian +=
            _controlData->_legController->commands[i].kpCartesian * (_controlData->_legController->commands[i].pDes - _controlData->_legController->data[i].p);

        tau_Cartesian +=
            _controlData->_legController->commands[i].kdCartesian * (_controlData->_legController->commands[i].vDes - _controlData->_legController->data[i].v);

        // torque
        tau_Cartesian = _controlData->_legController->data[i].J.transpose() * tau_Cartesian;

        // Vec3<double> tau_PD = _controlData->_legController->commands[i].kpJoint *
        //                           (_controlData->_legController->commands[i].qDes - _controlData->_legController->data[i].q) +
        //                       _controlData->_legController->commands[i].kdJoint *
        //                           (_controlData->_legController->commands[i].qdDes - _controlData->_legController->data[i].qd);

        // _controlData->_legController->commands[i].tau = legTorque + tau_Cartesian; //tau_ff + tau_Cartesian;

        _controlData->_legController->commands[i].tau = tau_ff + tau_Cartesian; // full torque

        // for (int j = 0; j<3; j++){
        //     lowCmd.motorCmd[i*3+j].torque = _controlData->_legController->commands[i].tau(j);

        //  //std::cout << "motor torque cmd: " << lowCmd.motorCmd[i*3+j].torque << std::endl;
        //  //std::cout << commands[i].tau(j) << std::endl;
        // }

    }

    
    // // limit torque --> limit current run through motor (Unitree specs)

    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 3; j++)
        {
            if (_controlData->_legController->commands[i].tau[j] >= _joint_torque_max)
            {
                _controlData->_legController->commands[i].tau[j] = _joint_torque_max;
            }

            if (_controlData->_legController->commands[i].tau[j] <= -_joint_torque_max)
            {
                _controlData->_legController->commands[i].tau[j] = -_joint_torque_max;
            }
        }
    }


    for (int i = 0; i < 4; i++)
    {
        Vec3<double> tau_PD = _controlData->_legController->commands[i].kpJoint *
                            (_controlData->_legController->commands[i].qDes - _controlData->_legController->data[i].q) +
                              _controlData->_legController->commands[i].kdJoint *
                            (_controlData->_legController->commands[i].qdDes - _controlData->_legController->data[i].qd);
        // send torque to motor
        for (int j = 0; j < 3; j++)
        {
            lowCmd.motorCmd[i * 3 + j].position = _controlData->_legController->commands[i].qDes[j];
            lowCmd.motorCmd[i * 3 + j].velocity = _controlData->_legController->commands[i].qdDes[j];
            lowCmd.motorCmd[i * 3 + j].torque = _controlData->_legController->commands[i].tau[j] - 1 * tau_PD[j]; // legTorque[j];
            lowCmd.motorCmd[i * 3 + j].positionStiffness = _controlData->_legController->commands[i].kpJoint(j, j);
            lowCmd.motorCmd[i * 3 + j].velocityStiffness = _controlData->_legController->commands[i].kdJoint(j, j);
        }
    }

    // Send command to motor servo
    sendServoCmd();
}


void JumpingObj::computeFullTorquesAndSend_constraints_v1() // voltage only
{

    double Kt = 0.118; // 4/34
    double torque_motor_max = 4;
    double max_ms = 1700 * 2 * 3.14 / 60;
    double min_ms = 940 * 2 * 3.14 / 60;
    double _gear_ratio = 8.5;

    double max_js = max_ms/ _gear_ratio;
    double min_js = min_ms/ _gear_ratio;

    double min_ts = 0.2 * _gear_ratio; // # min torque from plot
    double max_ts = torque_motor_max * _gear_ratio; // # max torque from plot

    // the parameters of green lines, converted to joint instead of motor
    double alpha_joint = (min_js - max_js)/(max_ts - min_ts); 
    double bj= min_js-alpha_joint * max_ts; 

    double _voltage_max = 21.5;
    double _current_max = 59.99;
    
    double _joint_vel_limit = 21;
    double _joint_torque_max = 33.5;
    // double _R_motor = 25 * Kt * Kt;
    double _R_motor = 0.638;
    double voltage[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // voltage for all joints
    double current[12] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // current for all joints
    double total_current = 0;
    double total_torque = 0;
    double total_power = 0 ;
    // cout << "max_js:" << max_js << std::endl;


    for (int i = 0; i < 4; i++)
    {

        Vec3<double> legTorque = -1.0 * _controlData->_legController->commands[i].tau;
        // std::cout << "commmand" << commands[i].tau << std::endl;
        //  forceFF

        Vec3<double> tau_ff = legTorque + _controlData->_legController->commands[i].kpJoint * (_controlData->_legController->commands[i].qDes - _controlData->_legController->data[i].q) + _controlData->_legController->commands[i].kdJoint * (_controlData->_legController->commands[i].qdDes - _controlData->_legController->data[i].qd);

        // Jacobian already updated!
        Vec3<double> tau_Cartesian(0, 0, 0);
        tau_Cartesian +=
            _controlData->_legController->commands[i].kpCartesian * (_controlData->_legController->commands[i].pDes - _controlData->_legController->data[i].p);

        tau_Cartesian +=
            _controlData->_legController->commands[i].kdCartesian * (_controlData->_legController->commands[i].vDes - _controlData->_legController->data[i].v);

        // torque
        tau_Cartesian = _controlData->_legController->data[i].J.transpose() * tau_Cartesian;

        // Vec3<double> tau_PD = _controlData->_legController->commands[i].kpJoint *
        //                           (_controlData->_legController->commands[i].qDes - _controlData->_legController->data[i].q) +
        //                       _controlData->_legController->commands[i].kdJoint *
        //                           (_controlData->_legController->commands[i].qdDes - _controlData->_legController->data[i].qd);

        // _controlData->_legController->commands[i].tau = legTorque + tau_Cartesian; //tau_ff + tau_Cartesian;

        _controlData->_legController->commands[i].tau = tau_ff + tau_Cartesian; // full torque

        // for (int j = 0; j<3; j++){
        //     lowCmd.motorCmd[i*3+j].torque = _controlData->_legController->commands[i].tau(j);

        //  //std::cout << "motor torque cmd: " << lowCmd.motorCmd[i*3+j].torque << std::endl;
        //  //std::cout << commands[i].tau(j) << std::endl;
        // }

    }
 

    // Limit joint velocity via torque adjustment

    // for (int i=0; i<4; i++){
    //     for (int j=0; j<3; j++){
    //         if(abs(_controlData->_legController->data[i].qd[j]) >= _joint_vel_limit*0.75)
    //         {
    //             _controlData->_legController->commands[i].tau[j] = (_controlData->_legController->data[i].qd[j]-bj)/alpha_joint;
    //         }
    //     }
    // }


    // NOTE: Limit total current / upper power, voltage, lower power

    // Compute voltage, limit voltage via limit torque

    for (int i =0; i<4; i++){

        for (int j = 0; j < 3; j++)
        {
            voltage[i * 3 + j] = _controlData->_legController->commands[i].tau(j) * _R_motor / (Kt * _gear_ratio) + _controlData->_legController->data[i].qd[j] * _gear_ratio * Kt;
            
            // cout << "voltage:" << voltage[i * 3 + j] << std::endl;
            if (voltage[i * 3 + j] > _voltage_max)
            {
                _controlData->_legController->commands[i].tau[j] = (_voltage_max - 1.0 * _controlData->_legController->data[i].qd[j] * _gear_ratio * Kt) * (Kt * _gear_ratio / _R_motor);
            }
            if (voltage[i * 3 + j] < -1.0 * _voltage_max)
            {
                _controlData->_legController->commands[i].tau[j] = (-1.0 * _voltage_max - 1.0 * _controlData->_legController->data[i].qd[j] * _gear_ratio * Kt) * (Kt * _gear_ratio / _R_motor);
            }
        }


    }

    // limit torque --> limit current run through motor (Unitree specs)

    for (int i = 0; i < 4; i++){
        for (int j = 0; j < 3; j++)
        {
            if (_controlData->_legController->commands[i].tau[j] >= _joint_torque_max)
            {
                _controlData->_legController->commands[i].tau[j] = _joint_torque_max;
            }

            if (_controlData->_legController->commands[i].tau[j] <= -_joint_torque_max)
            {
                _controlData->_legController->commands[i].tau[j] = -_joint_torque_max;
            }
        }
    }



    for (int i = 0; i < 4; i++)
    {
        Vec3<double> tau_PD = _controlData->_legController->commands[i].kpJoint *
                            (_controlData->_legController->commands[i].qDes - _controlData->_legController->data[i].q) +
                              _controlData->_legController->commands[i].kdJoint *
                            (_controlData->_legController->commands[i].qdDes - _controlData->_legController->data[i].qd);
        // send torque to motor
        for (int j = 0; j < 3; j++)
        {
            lowCmd.motorCmd[i * 3 + j].position = _controlData->_legController->commands[i].qDes[j];
            lowCmd.motorCmd[i * 3 + j].velocity = _controlData->_legController->commands[i].qdDes[j];
            lowCmd.motorCmd[i * 3 + j].torque = _controlData->_legController->commands[i].tau[j] - 1 * tau_PD[j]; // legTorque[j];
            lowCmd.motorCmd[i * 3 + j].positionStiffness = _controlData->_legController->commands[i].kpJoint(j, j);
            lowCmd.motorCmd[i * 3 + j].velocityStiffness = _controlData->_legController->commands[i].kdJoint(j, j);
        }
    }

    // Send command to motor servo
    sendServoCmd();
}

/**
 * Process observation like in VecNormalize, need mean and std dev (read in from files)
 */
void JumpingObj::normalizeObservation(std::vector<float> &obs)
{
    for (int i = 0; i < obs_len; i++)
    {
        // cout <<  "rl_obs(i) = " << rl_obs.data[i] << endl;
        // cout << "obs_rms_mean(i) = " << obs_rms_mean[i] << endl;
        // cout << "obs_rms_var(i) = " << obs_rms_var[i] << endl;
        // clip to limits
        if (rl_obs.data[i] > obs_high[i]) //(rl_obs.data[i] > obs_high[i])
            rl_obs.data[i] = obs_high[i];
        else if (rl_obs.data[i] < obs_low[i])
            rl_obs.data[i] = obs_low[i];
        // normalize
        // (obs - self.obs_rms.mean) / np.sqrt(self.obs_rms.var + self.epsilon),
        obs[i] = (rl_obs.data[i] - obs_rms_mean[i]) / sqrt(obs_rms_var[i] + 1e-8);

        // else
        //     obs[i] = rl_obs.data[i];

        // // clip to limits
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

/**
 * full RL method: get obs, scale, send to NN, apply leg commands
 */
// void JumpingObj::computeAction(int Iter,  std::vector<double> init_final,  std::vector<double> contactState)
// {
//     // construct obs
//     // getObservation();
//     // getRealObservation();
//     // getMinimalObservation_hist(Iter); // (61+72*5+4)*5
//     // getObservation_hist_v2(Iter, init_final); // remove torque+ add initial, final config
//     getObservation_hist_v3(Iter, init_final, contactState); // remove torque+ add initial, final config, use future ref at 1,5,17 only
//     // cout<<" code pass 1:" << endl;
//     // normalize observation
//     // std::cout << "norm obs" << std::endl;
//     // std::vector<float> obs(obs_len);
//     // cout << "obs_len :" << obs_len << endl;

//     // cout <<"chuong:" << chuong << std::endl;

//     // Normalization

//     std::vector<float> obs(obs_len);
//     for(int i = 0; i < obs_len; i++){
//         // cout <<  "rl_obs(i) = " << rl_obs.data[i] << endl;
//         // cout << "obs_rms_mean(i) = " << obs_rms_mean[i] << endl;
//         // cout << "obs_rms_var(i) = " << obs_rms_var[i] << endl;
//         // clip to limits
//         if (rl_obs.data[i] > obs_high[i]) //(rl_obs.data[i] > obs_high[i])
//             rl_obs.data[i] = obs_high[i];
//         else if (rl_obs.data[i] < obs_low[i])
//             rl_obs.data[i] = obs_low[i];
//         // normalize
//         // (obs - self.obs_rms.mean) / np.sqrt(self.obs_rms.var + self.epsilon),
//         obs[i] = (rl_obs.data[i] - obs_rms_mean[i] ) / sqrt( obs_rms_var[i] + 1e-8 );

//     }

//     // normalizeObservation(obs);
//     std::cout << "checking obs len... "  << std::endl;
//     cout<< "obs_length=" << obs_len << endl;
//     // for (int i = 0; i < 2310 ; i++){
//     //     cout << i << " " << obs[i]  << endl;
//     // }

//     // cout<<" pass 4:" << endl;
//     // get action from policy network
//     // set observation
//     // std::cout << "set data" << std::endl;
// //    observation.set_data(obs);
//     observation = cppflow::tensor(obs, {1, obs_len});

//     cout<< "observation" << observation << endl;

//     // run model and set action
//     // std::cout << "run model" << std::endl;
//     // std::cout << "======================================================================" << std::endl;
//     // std::cout << "checking obs len... "  << std::endl;
//     // for (int i = 0; i < obs_len ; i++){
//     //     std::cout << i << " " << obs[i]  << std::endl;
//     // }
//     // std::cout << "checking act... " << std::endl;
//     // for (int i = 0; i < act_len ; i++){
//     //     std::cout << i << " " << action[i] << << std::endl;
//     // }
// //    model.run({&observation}, action);

//     auto action = model({{"default_policy/obs:0", observation}},{"default_policy/model/fc_out/BiasAdd:0"})[0];
//     // auto action = model({{"default_policy/obs:0", observation}},{"default_policy/cond_1/Merge:0"})[0];

//     cout<< "action:" << action <<endl;

//     // std::cout << "post model run" << std::endl;
//     // act_arr_vec.clear();

//     // scale action:
//     int i = 0;
//     for (float f : action.get_data<float>()) {
//         float a = f;
//         // clip
//         if (a > 1)
//             a = 1.0;
//         else if (a < -1)
//             a = -1.0;

//         // last_action_rl[i] = a;
//         std::cout << "last_action_rl:" << last_action_rl[i] << std::endl;
//         std::cout << "action:" << a*0.1 << std::endl;

//         if (enable_action_filter){
//             double lam = 0.1;
//             double temp_a = a;
//             a = lam*a + (1-lam) * last_action_rl[i];
//             last_action_rl[i] = temp_a;
//         }

//         a = a * .1;

//         // // // scale
//         std::cout << "act_low[i]: " << act_low[i] << ", after filter a =" << a << " , act_high[i]: " << act_high[i] << std::endl;
//         // a = act_low[i] + 0.5 * (a+1) * (act_high[i] - act_low[i]);
//         // std::cout << "new a: " << a << std::endl;
//         // set RLAct array
//         // rl_act.data[i] = a;

//         act_arr[i] = a;
//         // act_arr_vec.push_back((double) a);
//         i++;
//         if (i >= 12){
//             break;
//         }
//     }

//     if (Iter > 900){ // flight idx+100
//         for (int j =0; j<12; j++){
//             act_arr[j]=0;
//         }
//     }
//     // sets legCommands
//     //processAndSetLegCommandsFromRL(rl_act);
//     // generalSetLegCommandsRL();

// }

void JumpingObj::computeAction_2D(int Iter, std::vector<double> init_final, std::vector<double> contactState)
{
    // construct obs
    // getObservation();
    // getRealObservation();
    // getMinimalObservation_hist(Iter); // (61+72*5+4)*5
    // getObservation_hist_v2(Iter, init_final); // remove torque+ add initial, final config
    get2DObservation_hist(Iter, init_final, contactState); // remove torque+ add initial, final config, use future ref at 1,5,17 only
    // cout<<" code pass 1:" << endl;
    // normalize observation
    // std::cout << "norm obs" << std::endl;
    // std::vector<float> obs(obs_len);
    // cout << "obs_len :" << obs_len << endl;

    // cout <<"chuong:" << chuong << std::endl;

    // Normalization

    std::vector<float> obs(obs_len);
    for (int i = 0; i < obs_len; i++)
    {
        // cout <<  "rl_obs(i) = " << rl_obs.data[i] << endl;
        // cout << "obs_rms_mean(i) = " << obs_rms_mean[i] << endl;
        // cout << "obs_rms_var(i) = " << obs_rms_var[i] << endl;
        // clip to limits
        if (rl_obs.data[i] > obs_high[i]) //(rl_obs.data[i] > obs_high[i])
            rl_obs.data[i] = obs_high[i];
        else if (rl_obs.data[i] < obs_low[i])
            rl_obs.data[i] = obs_low[i];
        // normalize
        // (obs - self.obs_rms.mean) / np.sqrt(self.obs_rms.var + self.epsilon),
        obs[i] = (rl_obs.data[i] - obs_rms_mean[i]) / sqrt(obs_rms_var[i] + 1e-8);
    }

    // normalizeObservation(obs);
    std::cout << "checking obs len... " << std::endl;
    // cout<< "obs_length=" << obs_len << endl;
    for (int i = 0; i < 280; i++)
    {
        cout << i << " " << obs[i] << endl;
    }

    // cout<<" pass 4:" << endl;
    // get action from policy network
    // set observation
    // std::cout << "set data" << std::endl;
    //    observation.set_data(obs);
    observation = cppflow::tensor(obs, {1, obs_len});

    // cout<< "observation" << observation << endl;

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
    //    model.run({&observation}, action);

    auto action = model({{"default_policy/obs:0", observation}}, {"default_policy/model/fc_out/BiasAdd:0"})[0];
    // auto action = model({{"default_policy/obs:0", observation}},{"default_policy/cond_1/Merge:0"})[0];

    // cout<< "action:" << action <<endl;

    // std::cout << "post model run" << std::endl;
    // act_arr_vec.clear();

    // scale action:
    int i = 0;
    for (float f : action.get_data<float>())
    {
        float a = f;
        // clip
        if (a > 1)
            a = 1.0;
        else if (a < -1)
            a = -1.0;

        // last_action_rl[i] = a;
        std::cout << "last_action_rl:" << last_action_rl[i] << std::endl;
        std::cout << "action:" << a * 0.1 << std::endl;

        if (enable_action_filter)
        {
            double lam = 0.9;
            double temp_a = a;
            a = lam * a + (1 - lam) * last_action_rl[i];
            last_action_rl[i] = temp_a;
        }

        a = a * .1;

        // // // scale
        std::cout << "act_low[i]: " << act_low[i] << ", after filter a =" << a << " , act_high[i]: " << act_high[i] << std::endl;
        // a = act_low[i] + 0.5 * (a+1) * (act_high[i] - act_low[i]);
        // std::cout << "new a: " << a << std::endl;
        // set RLAct array
        // rl_act.data[i] = a;

        act_arr[i] = a;
        // act_arr_vec.push_back((double) a);
        i++;
        if (i >= 4)
        {
            break;
        }
    }

    if (Iter > 800)
    { // flight idx+100
        for (int j = 0; j < 4; j++)
        {
            act_arr[j] = 0;
        }
    }
    // sets legCommands
    // processAndSetLegCommandsFromRL(rl_act);
    // generalSetLegCommandsRL();
}

/**
 * GENERAL set RL actions to leg commands, for general action space, depending on act_len
 */
// void JumpingObj::generalSetLegCommandsRL()
// {

//     // assume already scaled, so just set pDes, etc.
//     auto& seResult = _controlData->_stateEstimator->getResult();
//     Vec3<double> feedforwardForce;
//     int forcesIdx = 12;

//     // check contact bools, only apply force if in contact
//     double inContact[4];
//     for (int i = 0; i < 4; i++){
//         if (lowState.footForce[i] > 0){
//             inContact[i] = 1; // positive force, so in contact
//         }
//         else{
//             inContact[i] = 0; // in air
//         }
//     }

//     // set high level commands, may be used several times while waiting for next high level command from pybullet
//     for (int i = 0; i < 4; i++){
//         // rotate world frame force into body frame
//         if (inContact[i]){
//             // IMPEDANCE action space
//             if (act_len == 16){
//                 feedforwardForce << 0, 0, act_arr[forcesIdx+i];
//             }
//             else {
//                 // IMPEDANCE_3D_FORCE action space
//                 feedforwardForce << act_arr[forcesIdx+i*3], act_arr[forcesIdx+i*3+1], act_arr[forcesIdx+i*3+2];
//             }
//             feedforwardForce = seResult.rBody.transpose() * feedforwardForce; //msg.data[forcesIdx+i];
//         }
//         else {
//              feedforwardForce << 0, 0, 0;
//         }
//         for (int j = 0; j < 3; j++){
//             _controlData->_legController->commands[i].pDes[j] = act_arr[i*3+j];
//             _controlData->_legController->commands[i].vDes[j] = 0;
//             _controlData->_legController->commands[i].kpCartesian(j,j) = 500;
//             _controlData->_legController->commands[i].kdCartesian(j,j) = 10;
//             _controlData->_legController->commands[i].feedforwardForce[j] = feedforwardForce[j];
//         }
//     }

//     _controlData->_legController->updateCommand();

//     hasData = 1;

// }

/**
 * Postprocess raw actions that are in [-1,1] to leg commands, this for IMPEDANCE action space
 */
void JumpingObj::processAndSetLegCommandsFromRL(const laikago_msgs::RLAct &msg)
{

    // assume already scaled, so just set pDes, etc.
    auto &seResult = _controlData->_stateEstimator->getResult();
    Vec3<double> feedforwardForce;
    int forcesIdx = 12;

    // check contact bools, only apply force if in contact
    double inContact[4];
    for (int i = 0; i < 4; i++)
    {
        if (lowState.footForce[i] > 0)
        {
            inContact[i] = 1; // positive force, so in contact
        }
        else
        {
            inContact[i] = 0; // in air
        }
    }

    // set high level commands, may be used several times while waiting for next high level command from pybullet
    for (int i = 0; i < 4; i++)
    {
        // rotate world frame force into body frame
        if (inContact[i])
        {
            // Vec3<double> feedforwardForce(0, 0, msg.data[forcesIdx+i]);
            feedforwardForce << 0, 0, msg.data[forcesIdx + i];
            feedforwardForce = seResult.rBody.transpose() * feedforwardForce; // msg.data[forcesIdx+i];
        }
        else
        {
            feedforwardForce << 0, 0, 0;
        }
        for (int j = 0; j < 3; j++)
        {
            _controlData->_legController->commands[i].pDes[j] = msg.data[i * 3 + j];             // legCmds[i].pDes[j]; //pDesLeg;
            _controlData->_legController->commands[i].vDes[j] = 0;                               // msg.legCmds[i].vDes[j];
            _controlData->_legController->commands[i].kpCartesian(j, j) = 500;                   // msg.legCmds[i].kpCartesianDiag[j];
            _controlData->_legController->commands[i].kdCartesian(j, j) = 10;                    // msg.legCmds[i].kdCartesianDiag[j];
            _controlData->_legController->commands[i].feedforwardForce[j] = feedforwardForce[j]; // msg.legCmds[i].feedforwardForce[j];
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
void JumpingObj::setupTensorflow()
{
    std::cout << "restore TF model..." << std::endl;
    // model.init();
    //  model.restore("/home/guillaume/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/"
    //                  "usc_learning/learning/rllib/exported/my_model");
    //  model.restore(tf_root + "variables/variables");
    //  model.restore(tf_root + "model");
    model = cppflow::model(tf_root + "my_model");
    std::cout << "...done" << std::endl;
}

/**
 * Query tensorflow service as test from pybullet.
 */
bool JumpingObj::queryTensorflowModelService(laikago_msgs::TFServiceRequest &msg, laikago_msgs::TFServiceResponse &response)
{
    // set observation
    // observation.set_data(msg.rlObs.data.cast<float>());
    std::vector<float> data(obs_len);
    for (int i = 0; i < obs_len; i++)
    {
        data[i] = msg.rlObs.data[i];
    }
    // observation.set_data(data);
    observation = cppflow::tensor(data);
    // run model and set action
    // model.run({&observation}, action);
    auto action = model({{"default_policy/obs:0", observation}}, {"default_policy/model/fc_out/BiasAdd:0"})[0];
    // set action in response and send
    // response.rlAct.data = action;
    int idx = 0;
    for (float f : action.get_data<float>())
    {
        response.rlAct.data[idx] = f;
        idx++;
    }

    return true;
}

/**
 * Load VecNormalize params (mean/var) and observation/action high/low
 */
bool JumpingObj::loadRLSpaceParams()
{
    // Read observation params
    std::ifstream vFile;
    // vFile.open("/home/guillaume/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/"
    //                 "usc_learning/learning/rllib/exported/vecnorm_params.csv");
    vFile.open(tf_root + "vecnorm_params.csv");
    // If cannot open the file, report an error
    if (!vFile.is_open())
    {
        std::cout << "Could not open vecnorm params file" << std::endl;
        return false;
    }
    cout << "Reading vecnorm params..." << endl;

    string line;
    int index = 0;
    int vecnorm_len = 0;

    while (getline(vFile, line))
    {
        // Line[0] has the observation mean
        if (index == 0)
        {
            stringstream ss(line);
            float val;
            while (ss >> val)
            {
                obs_rms_mean.push_back(val);
                // cout << "obs_rms_mean " << val << endl;
                if (ss.peek() == ',')
                    ss.ignore();
                vecnorm_len += 1;
            }
        }
        // Line[1] has the observation variance
        if (index == 1)
        {
            stringstream ss(line);
            float val;
            while (ss >> val)
            {
                obs_rms_var.push_back(val);
                // cout << "obs_rms_var " << val << endl;
                if (ss.peek() == ',')
                    ss.ignore();
            }
        }
        // Line[2] has the observation high
        if (index == 2)
        {
            stringstream ss(line);
            float val;
            while (ss >> val)
            {
                obs_high.push_back(val);
                // cout << "obs_high " << val << endl;
                if (ss.peek() == ',')
                    ss.ignore();
            }
        }
        // Line[3] has the observation low
        if (index == 3)
        {
            stringstream ss(line);
            float val;
            while (ss >> val)
            {
                obs_low.push_back(val);
                // cout << "obs_low " << val << endl;
                if (ss.peek() == ',')
                    ss.ignore();
            }
        }

        index++;
    }
    vFile.close();

    cout << "Observation space is " << vecnorm_len << endl;
    cout << "...done" << endl;

    // Read the action space file
    std::ifstream aFile;
    // aFile.open("/home/guillaume/Documents/Research/DRL/Reinforcement-Learning-for-Quadruped-Robots/"
    //                 "usc_learning/learning/rllib/exported/action_space.csv");
    aFile.open(tf_root + "action_space.csv");
    // If cannot open the file, report an error
    if (!aFile.is_open())
    {
        std::cout << "Could not open action space file" << std::endl;
        return false;
    }
    cout << "Reading action space params..." << endl;
    index = 0;

    while (getline(aFile, line))
    {
        // Line[0] has the action upper range
        if (index == 0)
        {
            stringstream ss(line);
            float val;
            while (ss >> val)
            {
                act_high.push_back(val);
                // cout << "val high " << val << endl;
                if (ss.peek() == ',')
                    ss.ignore();
            }
        }
        // Line[1] has the action lower range
        if (index == 1)
        {
            stringstream ss(line);
            float val;
            while (ss >> val)
            {
                act_low.push_back(val);
                // cout << "val low " << val << endl;
                if (ss.peek() == ',')
                    ss.ignore();
            }
        }
        index++;
    }
    aFile.close();

    return true;
}

/*
 * Load optimal traj from MATLAB
 * Read the trajectory file, this assumes it is ROW order, i.e. row 0 is the first full state (x), row 1 is (z), etc.
 */
bool JumpingObj::readTraj(std::string filename)
{

    std::ifstream trajFile;
    trajFile.open(filename); //"src/traj_opt/trajs/opt_traj.txt");
    // If cannot open the file, report an error
    if (!trajFile.is_open())
    {
        std::cout << "Could not open optimal trajectory file." << std::endl;
        return false;
    }
    cout << "Reading optimal trajectory... " << filename << endl;

    // std::vector<std::vector<double>> x_opt;
    //  clear vector in case already loaded some
    x_opt.clear();
    std::string tempstr;
    double tempdouble;
    // char delimiter;
    full_opt_N = 0;
    // opt_state_space = 0;
    int idx = 0;

    while (getline(trajFile, tempstr))
    {
        std::istringstream iss(tempstr);
        std::vector<double> tempv;
        // opt_state_space = 0;

        while (iss >> tempdouble)
        {
            opt_state_space += 1;
            tempv.push_back(tempdouble);
            if (iss.peek() == ',')
                iss.ignore();
            // iss >> delimiter;
        }
        x_opt.push_back(tempv);
        full_opt_N += 1;
        idx += 1;
    }

    // check input
    // for (const auto &row : x_opt)
    // {
    //   for (const auto &col : row)
    //       std::cout << col << ", ";
    //   std::cout << "\n";
    // }

    // full_opt_N = x_opt[0].size();

    cout << "Opt_state_space:" << opt_state_space << endl;

    cout << "Traj is " << full_opt_N << " knot points." << endl;
    trajFile.close();
    cout << "...done." << endl;
    return true;
}