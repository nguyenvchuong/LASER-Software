#include "../include/FSM.h"
#include "ros/ros.h"


FSM_State::FSM_State(ControlFSMData* _controlFSMData):
_data(_controlFSMData),
Cmpc(0.001, 30)
{
    std::cout<< "zero transition data" << std::endl;
    transitionData.zero();
    std::cout << "Initialize FSM" << std::endl;
}

/*void FSM_State::runBalanceController(){
    // reset forces and steps to 0
    footFeedForwardForces = Mat34<double>::Zero();
    footstepLocations = Mat34<double>::Zero();
    double minForce = 25;
    double maxForce = 500;
    double contactStateScheduled[4] ={1, 1, 1, 1};
    for (int i = 0; i < 4; i++) {
        contactStateScheduled[i] =
        _data->_gaitScheduler->gaitData.contactStateScheduled(i);
    }

    double minForces[4];  // = {minForce, minForce, minForce, minForce};
    double maxForces[4];  // = {maxForce, maxForce, maxForce, maxForce};
    for (int leg = 0; leg < 4; leg++) {
        minForces[leg] = contactStateScheduled[leg] * minForce;
        maxForces[leg] = contactStateScheduled[leg] * maxForce;
    }

    double COM_weights_stance[3] = {1, 1, 10};
    double Base_weights_stance[3] = {20, 20, 20};
    double pFeet[12], p_des[3], p_act[3], v_des[3], v_act[3], O_err[3], rpy[3],
      omegaDes[3];
    double se_xfb[13];
    double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];

    for (int i = 0; i < 4; i++) {
        se_xfb[i] = _data->_stateEstimator->getResult().orientation(i);
    }
    // se_xfb[3] = 1.0;
    for (int i = 0; i < 3; i++) {
        rpy[i] = _data->_stateEstimator->getResult().rpy(i);
        p_des[i] = _data->_stateEstimator->getResult().position(i);
        p_act[i] = _data->_stateEstimator->getResult().position(i);
        omegaDes[i] = _data->_stateEstimator->getResult().omegaBody(i);
        v_act[i] = _data->_stateEstimator->getResult().vBody(i);
        v_des[i] = _data->_stateEstimator->getResult().vBody(i);

        se_xfb[4 + i] = _data->_stateEstimator->getResult().position(i);
        se_xfb[7 + i] = _data->_stateEstimator->getResult().omegaBody(i);
        se_xfb[10 + i] = _data->_stateEstimator->getResult().vBody(i);

    // Set the translational and orientation gains
        kpCOM[i] = 30;    //_data->controlParameters->kpCOM(i);
        kdCOM[i] =  10; //_data->controlParameters->kdCOM(i);
        kpBase[i] = 80;   //_data->controlParameters->kpBase(i);
        kdBase[i] = 50; //  _data->controlParameters->kdBase(i);
    }

    Vec3<double> pFeetVec;
    Vec3<double> pFeetVecCOM;


    // Get the foot locations relative to COM
     for (int leg = 0; leg < 4; leg++) {
    computeLegJacobianAndPosition(_data->_legController->data[leg].q,
                                  (Mat3<double>*)nullptr, &pFeetVec, 1);
    //pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                  //(_data->_quadruped->getHipLocation(leg) + pFeetVec);

    pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                  (_data->_quadruped->getHipLocation(leg) + _data->_legController->data[leg].p);


    pFeet[leg * 3] = pFeetVecCOM[0];
    pFeet[leg * 3 + 1] = pFeetVecCOM[1];
    pFeet[leg * 3 + 2] = pFeetVecCOM[2];
  }
  
  balanceController.set_alpha_control(0.01);
  balanceController.set_friction(0.2);
  balanceController.set_mass(19.0);
  balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
  balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
  balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
  balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
  balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                                      O_err, _data->_stateEstimator->getResult().rpy(2));

  double fOpt[12];
  balanceController.solveQP_nonThreaded(fOpt);

  // Publish the results over ROS
  // balanceController.publish_data_lcm();

  // Copy the results to the feed forward forces
  for (int leg = 0; leg < 4; leg++) {
    footFeedForwardForces.col(leg) << fOpt[leg * 3], fOpt[leg * 3 + 1],
        fOpt[leg * 3 + 2];

    _data->_legController->commands[leg].feedforwardForce = footFeedForwardForces.col(leg);
    
  }
  std::cout << footFeedForwardForces << std::endl;
  _data->_legController->updateCommand();
}
*/


void FSM_State::QPstand(){
    ofstream myfile;
    ofstream QP;
    ofstream z_pos;
    ofstream b_des;
    myfile.open ("ori.txt");
    QP.open("QPsolution.txt");
    z_pos.open("zPos.txt");
    b_des.open("b_des_z.txt");
    ros::Rate rate(1000);
    // reset forces and steps to 0
    footFeedForwardForces = Mat34<double>::Zero();
    footstepLocations = Mat34<double>::Zero();
    double minForce = 5;
    double maxForce = 500;
    double contactStateScheduled[4] = {1, 1, 1, 1};

    double minForces[4] = {minForce, minForce, minForce, minForce};
    double maxForces[4] = {maxForce, maxForce, maxForce, maxForce};

    //double COM_weights_stance[3] = {50, 50, 50};
    //double Base_weights_stance[3] = {500, 150, 30};// for QP locomotion
    double COM_weights_stance[3] = {5, 5, 10};
    double Base_weights_stance[3] = {10, 10, 20};
    double pFeet[12], p_act[3], v_act[3], O_err[3], rpy[3],v_des[3], p_des[3],
      omegaDes[3];
    double se_xfb[13];
    double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];
    double b_control[6];
    int counter = 0;
       // ros::spinOnce();
       _data->_legController->updateData();
       _data->_stateEstimator->run();
    // se_xfb[3] = 1.0; 
    //  int during = 10;
    // position & rpy desired
    for(int i = 0; i < 3; i++){
      p_des[i] = _data->_stateEstimator->getResult().position(i); 
      v_des[i] = 0;
      omegaDes[i] = 0;
      rpy[i] = 0;
    }
       rpy[2] = _data->_stateEstimator->getResult().rpy(2);
      p_des[2] = 0.3;
     bool firstLocoRun = true;
      // p_des[2] = 0.4;
    //double v_des[3] = {0, 0, 0};
    while(ros::ok()) {
     //std::cout << ros::Time::now() << std::endl;

       _data->_legController->updateData();
       _data->_stateEstimator->run();
      
      
      // for(int i =0; i<3; i++){
      //   myfile << _data->_stateEstimator->getResult().rpy(i);
      //   myfile << " " ;
      // }
  
    /*======================== MPC locomotion ====================*/
   if(counter > 1500){
      _data->_legController->updateData();
      _data->_stateEstimator->run();
      Vec3<double> v_des(0, 0, 0);
      double yaw_rate = 0;
      Cmpc.setGaitNum(2);
     if(fabs(_data->_stateEstimator->getResult().rpy[2]) > 0.01){
        if(_data->_stateEstimator->getResult().rpy[2] < 0)
          yaw_rate = 0.5;
      else yaw_rate = -0.5;
       _data->_desiredStateCommand->setStateCommands(v_des, yaw_rate);
     }
      // std::cout << "outside mpc" << _data->_stateEstimator->getResult().rBody << std::endl;
      //std::cout << "---------------------------------------\n";
      //std::cout << "iteration:" << counter-500 << std::endl;
     
       //std::cout << "walk" << std::endl;
       //v_des[0] = 0.1;
      // v_des[1] = -0.2;
        //Cmpc.setGaitNum(2);
       //v_des[1] = _data->_stateEstimator->getResult().vWorld(1);
       //yaw_rate = 1.2;
      if(counter > 3000){
        v_des[0] = 0.5;
       // v_des[1] = 0;
       // yaw_rate = 5;
      }
      // b_des << _data->_stateEstimator->getResult().position[0] << " ";
      // b_des << _data->_stateEstimator->getResult().position[1] << "\n ";

      // for(int i =0; i<3; i++){
      //    myfile << _data->_stateEstimator->getResult().rpy[i] << " ";
        
      //  }
      // myfile << "\n";

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
      //  v_des[1] = 0;
        yaw_rate = 0;
      }

      if(counter > 30000){
        yaw_rate = 0;
        Cmpc.setGaitNum(2);
      }

       _data->_desiredStateCommand->setStateCommands(v_des, yaw_rate);
      // if(counter > 3000){
      //   QP << _data->_desiredStateCommand->data.stateDes(5) << " ";
      //   QP << _data->_stateEstimator->getResult().rpy[2] << "\n";
      // }
     Cmpc.run(*_data);
      
    
     runQP = false;
    }

     // fstrem for plots
    //  for(int i = 0; i < 4; i++){
       
    //   //b_des << _data->_legController->commands[1].pDes.transpose(); // foot pos
    //   }
    //   QP << _data->_legController->commands[1].vDes[2] << " ";
    //   QP << _data->_legController->data[1].v[2] << "\n";
    // //  for(int i=0; i < 3; i++){
    // //   b_des << _data->_legController->commands[1].pDes[i] << " "; // foot pos
    // //  }
    //   b_des << _data->_legController->commands[1].pDes[2] << " ";
    //   b_des << _data->_legController->data[1].p[2] << "\n ";
    //   //b_des << _data->_legController->data[1].p[2] << "\n";
    //   for(int i =0; i<3; i++){
    //     myfile << _data->_legController->data[1].tau[i] << " ";
    //     z_pos << _data->_legController->commands[1].tau[i] << " ";
    //   }
    //  myfile << "\n";
    //   //z_pos << _data->_stateEstimator->getResult().position[2] << "\n";
    //   z_pos << "\n";
    //  QP << "\n";
    //  b_des << "\n";
    // std::cout << "iter: " << counter-500 << std::endl;
    // std::cout << "p error: \n" << _data->_legController->commands[1].pDes - _data->_legController->data[1].p << std::endl;
    // if(counter == 1800){
    //   runQP = true;
    // }
    
    /*======================== PD ========================*/
    // if(counter >= 1000){
    //   double side_sign[4] = {-1, 1, -1, 1};
    //   if(counter == 1000)
    //     std::cout << "start PD" << std::endl;
    //   for(int i = 0; i < 4; i++){
    //     _data->_legController->commands[i].kpCartesian = Vec3<double>(1000,1000,1000).asDiagonal();
    //     _data->_legController->commands[i].kdCartesian = Vec3<double>(30,30,30).asDiagonal();
    //     if(counter == 1000){
    //     _data->_legController->commands[i].feedforwardForce << 0, 0, 0;
    //     _data->_legController->commands[i].pDes = _data->_legController->data[i].p;
    //     }
    //   }
    //  // std::cout << "switch to PD" << std::endl;
    //   runQP = false;
    // }

    /*======================== QPlocomotion test =====================*/
    //  if (counter > 3000){
    //     double dt = 0.001;
    //     if(firstLocoRun){
    //       for(int i = 0; i < 3; i++){
    //         p_des[i] = _data->_stateEstimator->getResult().position(i); 
    //         v_des[i] = 0;
    //         omegaDes[i] = 0;
    //         rpy[i] = 0;
    //       }
    //       firstLocoRun = false;
    //     }

      
    //     for(int i = 0; i < 3; i++){
    //       kpCOM[i] = 100;
    //       kdCOM[i] = 50;
    //       kpBase[i] = 80;
    //       kdBase[i] = 50;
    //     p_des[i]= _data->_stateEstimator->getResult().position(i) + dt * v_des[i];
    //     }
    //     kpBase[1] = 150;
    //     kdBase[1] = 50;

    //     locomotionController.run(*_data);
    //     // update contact 
    //     for(int i = 0; i < 4; i++){
    //        contactStateScheduled[i] =
    //               _data->_stateEstimator->getResult().contactEstimate(i);
    //      }
    //      runQP = false;
    //   }
        
    /* =================================================================*/
    if(runQP){
      for (int i = 0; i < 4; i++) {
        se_xfb[i] = _data->_stateEstimator->getResult().orientation(i);
      }
      //std::cout << "quat_act " << std::endl;
      //std::cout << _data->_stateEstimator->getResult().orientation << std::endl;
      //std::cout << "rpy_act" << std::endl;
      //std::cout << _data->_stateEstimator->getResult().rpy << std::endl;
      //std::cout << "balance controller" << std::endl;
      //runBalanceController();
       // v_des[2] = 0.1;
      for (int i = 0; i < 3; i++) {
        //rpy[i] = 0;
        //rpy[i] = _data->_stateEstimator->getResult().rpy(i);
        //p_des[i] = _data->_stateEstimator->getResult().position(i);
        p_act[i] = _data->_stateEstimator->getResult().position(i);
        
        v_act[i] = _data->_stateEstimator->getResult().vBody(i);
        // v_des[i] = 0;
        //v_des[2] = 0.005;

        se_xfb[4 + i] = _data->_stateEstimator->getResult().position(i);
        se_xfb[7 + i] = _data->_stateEstimator->getResult().omegaBody(i);
        se_xfb[10 + i] = _data->_stateEstimator->getResult().vBody(i);
        
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

        pFeetVecCOM =  _data->_stateEstimator->getResult().rBody.transpose() *
        (_data->_quadruped->getHipLocation(leg) + _data->_legController->data[leg].p);


        pFeet[leg * 3] = pFeetVecCOM[0];
        pFeet[leg * 3 + 1] = pFeetVecCOM[1];
        pFeet[leg * 3 + 2] = pFeetVecCOM[2];
        //std::cout << "pFeet" << leg << std::endl;
      }
    //  myfile << "\n";
     // std::cout << j << std::endl;
     //std::cout << "run QP" << std::endl;
    balanceController.set_alpha_control(0.01);
    balanceController.set_friction(0.6);
    balanceController.set_mass(_data->_quadruped->mass);
    balanceController.set_wrench_weights(COM_weights_stance, Base_weights_stance);
    balanceController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
    balanceController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
    balanceController.SetContactData(contactStateScheduled, minForces, maxForces);
    balanceController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                                      O_err, _data->_stateEstimator->getResult().rpy(2));
   // balanceController.print_QPData();
    double fOpt[12];
    balanceController.solveQP_nonThreaded(fOpt);
    //balanceController.get_b_matrix(b_control);
    //b_des << b_control[2] << "\n";

  // Publish the results over ROS
  // balanceController.publish_data_lcm();

  // Copy the results to the feed forward forces
    
     //_data->_stateEstimator->run();


    for (int leg = 0; leg < 4; leg++) {
        footFeedForwardForces.col(leg) << fOpt[leg * 3], fOpt[leg * 3 + 1],
        fOpt[leg * 3 + 2]; // force in world frame, need to convert to body frame

        _data->_legController->commands[leg].feedforwardForce = _data->_stateEstimator->getResult().rBody.transpose() *
         footFeedForwardForces.col(leg);
        //_data->_stateEstimator->getResult().rBody.transpose() * footFeedForwardForces.col(leg); 
       // QP << _data->_legController->commands[leg].feedforwardForce[2] << " ";
    }
  
    //std::cout << j << std::endl;
    //QP << "\n";
    
    
    //_data->_legController->commands[3].feedforwardForce = _data->_legController->commands[2].feedforwardForce;
    //if(p_act[2] < 0.25){
     //  std::cout << "force" << std::endl;
    //std::cout << footFeedForwardForces << std::endl;
    //_data->_legController->updateCommand();

    // plots
    // if(counter>500){
    //  for(int i = 0; i < 4; i++){
    //    QP << _data->_legController->commands[i].feedforwardForce[2] << " ";
    //    //b_des << _data->_legController->commands[i].pDes[2] << " "; // foot pos
       
    //   }
    //   b_des << _data->_legController->commands[1].pDes[2] << " ";
    //   b_des << _data->_legController->data[1].p[2] << "\n ";
      
    //   for(int i =0; i<3; i++){
    //     //myfile << _data->_stateEstimator->getResult().rpy(i);
    //     //myfile << " " ;
    //      myfile << _data->_legController->commands[1].pDes[i] - _data->_legController->data[1].p[i] << " ";
    //   }
    //  myfile << "\n";
    //   z_pos << _data->_stateEstimator->getResult().position(2) << "\n";
    //  QP << "\n";
    //  //b_des << "\n";
    // }
    }

    _data->_legController->updateCommand();
    rate.sleep();
    //_data->_stateEstimator->run();
    //std::cout << "vBody:" << std::endl;
    //std::cout << _data->_stateEstimator->getResult().vBody << std::endl;
    //std::cout << "p_act_z" << std::endl;
    //std::cout << p_act[2] << std::endl;
    //z_pos << p_act[2] << " " << v_act[2] << "\n";
    //std::cout << "rbody" << std::endl;
    //std::cout << _data->_stateEstimator->getResult().rBody << std::endl;
    //std::cout << "omega" << std::endl;
    //std::cout << _data->_stateEstimator->getResult().omegaBody << std::endl;
    //std::cout << "joint torque" << std::endl;
    //for (int  i = 0; i<12; i++){
    //    std::cout << lowState.motorState[i].torque << std::endl;
    //}
    //if (p_des[2] - p_act[2] < 0.01){
    //  runBalanceController();
    //  break;
    //}
    counter++;
 
}
 
 b_des.close();
 z_pos.close();
 myfile.close();
 QP.close();
 std::cout << "stand up finished" << std::endl;
}





// least square solution test
/*
void FSM_State::runLeastSquare(){

    ofstream myfile;
    ofstream Sol;
    ofstream f_y;
   // ofstream z_pos;
  //  ofstream b_des;
    myfile.open ("ori_leastSqure.txt");
    Sol.open("leastSquareSol.txt");
    f_y.open("fy.txt");
   // b_des.open("b_des_z.txt");
     ros::Rate rate(1000);
    footFeedForwardForces = Mat34<double>::Zero();
    footstepLocations = Mat34<double>::Zero();

    double minForce = 15;
    double maxForce = 500;
    double contactStateScheduled[4] = {1, 1, 1, 1};

    double minForces[4] = {minForce, minForce, minForce, minForce};
    double maxForces[4] = {maxForce, maxForce, maxForce, maxForce};

    double pFeet[12], p_des[3], p_act[3], v_des[3], v_act[3], O_err[3], rpy[3],
      omegaDes[3];
    double se_xfb[13];
    double kpCOM[3], kdCOM[3], kpBase[3], kdBase[3];

     ros::spinOnce();
       _data->_legController->updateData();
       _data->_stateEstimator->run();

    for(int i = 0; i < 3; i++){
      p_des[i] = _data->_stateEstimator->getResult().position(i); 
      v_des[i] = 0;
      omegaDes[i] = 0;
      rpy[i] = 0;
      rpy[2] = _data->_stateEstimator->getResult().rpy(2);
    } 
    p_des[2] = 0.4;

   for(int j =0; j < 500; j++) {
      
      _data->_legController->updateData();
      _data->_stateEstimator->run();

      for (int i = 0; i < 4; i++) {
        se_xfb[i] = _data->_stateEstimator->getResult().orientation(i);
      }
    // se_xfb[3] = 1.0;
      for (int i = 0; i < 3; i++) {
        //rpy[i] = _data->_stateEstimator->getResult().rpy(i);
        //p_des[i] = _data->_stateEstimator->getResult().position(i);
        p_act[i] = _data->_stateEstimator->getResult().position(i);
        //omegaDes[i] = _data->_stateEstimator->getResult().omegaBody(i);
        v_act[i] = _data->_stateEstimator->getResult().vBody(i);
        //v_des[i] = _data->_stateEstimator->getResult().vBody(i);

        se_xfb[4 + i] = _data->_stateEstimator->getResult().position(i);
        se_xfb[7 + i] = _data->_stateEstimator->getResult().omegaBody(i);
        se_xfb[10 + i] = _data->_stateEstimator->getResult().vBody(i);

    // Set the translational and orientation gains
        kpCOM[i] = 30;    //_data->controlParameters->kpCOM(i);
        kdCOM[i] =  10; //_data->controlParameters->kdCOM(i);
        kpBase[i] = 30;   //_data->controlParameters->kpBase(i);
        kdBase[i] = 10; //  _data->controlParameters->kdBase(i);
      }
      kpCOM[2] = 30;
      kdCOM[2] = 10;
   
      Vec3<double> pFeetVec;
      Vec3<double> pFeetVecCOM;

      for(int i =0; i < 3; i++){
        myfile << _data->_stateEstimator->getResult().rpy(i);
        myfile << " " ;
      }
      myfile << "\n";
    // Get the foot locations relative to COM
       for (int leg = 0; leg < 4; leg++) {
        computeLegJacobianAndPosition(_data->_legController->data[leg].q,
                                  (Mat3<double>*)nullptr, &pFeetVec, leg);
        //pFeetVecCOM = _data->_stateEstimator->getResult().rBody.transpose() *
                  //(_data->_quadruped->getHipLocation(leg) + pFeetVec);

        pFeetVecCOM =  _data->_stateEstimator->getResult().rBody.transpose() *
        (_data->_quadruped->getHipLocation(leg) + _data->_legController->data[leg].p);


        pFeet[leg * 3] = pFeetVecCOM[0];
        pFeet[leg * 3 + 1] = pFeetVecCOM[1];
        pFeet[leg * 3 + 2] = pFeetVecCOM[2];
       // std::cout << "pFeet" << leg << std::endl;
       // std::cout << pFeetVecCOM << std::endl;
      }

    leastSquareController.set_alpha_control(0.01);
    leastSquareController.set_mass(19.0);
    leastSquareController.set_friction(0.4);
    //std::cout << "mass set \n";
    leastSquareController.set_PDgains(kpCOM, kdCOM, kpBase, kdBase);
    //std::cout << "PD gain set \n";
    leastSquareController.set_desiredTrajectoryData(rpy, p_des, omegaDes, v_des);
    //std::cout << "desired traj set \n";
    leastSquareController.SetContactData(contactStateScheduled, minForces, maxForces);
    //std::cout << "cotanct set \n"; 
    leastSquareController.updateProblemData(se_xfb, pFeet, p_des, p_act, v_des, v_act,
                                      O_err, _data->_stateEstimator->getResult().rpy(2));

    double fOpt[12];
  
    leastSquareController.solveLeastSquare(fOpt);

    std::cout << "finished" << std::endl;
    // balanceController.solveQP_nonThreaded(fOpt);

    // Publish the results over ROS
    // balanceController.publish_data_lcm();

    // Copy the results to the feed forward forces
    for (int leg = 0; leg < 4; leg++) {
      footFeedForwardForces.col(leg) << fOpt[leg * 3], fOpt[leg * 3 + 1],
      fOpt[leg * 3 + 2];

     _data->_legController->commands[leg].feedforwardForce = _data->_stateEstimator->getResult().rBody.transpose() *
                                                             footFeedForwardForces.col(leg);
      //Sol << _data->_legController->commands[leg].feedforwardForce[2] << " ";
      }
      //Sol << "\n";
    std::cout << footFeedForwardForces << std::endl;
    _data->_legController->updateCommand();
    rate.sleep();
  }

  Sol.close();
  myfile.close();
}
*/

void FSM_State::PDstand(){
  ros::Rate rate(1000);
  int counter = 0;
  
  // initial foot possition
  Mat34<double> init_foot_pos;
  _data->_legController->updateData();
  for(int i = 0; i < 4; i++){
    init_foot_pos.col(i) = _data->_legController->data[i].p;
  }
  double h = 0.4; // standup height
  double side_sign[4] = {-1, 1, -1, 1};
  Vec3<double> ff_force_world(0, 0, 0);

  while(ros::ok()){
     double progress = counter * 0.001;
     if(progress > 1.)  {progress = 1.;}

    _data->_legController->updateData();  
    _data->_stateEstimator->run();    

      for(int i = 0; i < 4; i++){
        _data->_legController->commands[i].kpCartesian = Vec3<double>(400,400,900).asDiagonal();
        _data->_legController->commands[i].kdCartesian = Vec3<double>(20,20,20).asDiagonal();

        _data->_legController->commands[i].pDes << 0, side_sign[i] * 0.083, 0;
        
        _data->_legController->commands[i].pDes[2] = -h*progress + (1. - progress) * init_foot_pos(2, i);
       // _data->_legController->commands[i].feedforwardForce = _data->_stateEstimator->getResult().rBody.transpose() * ff_force_world;
       // std::cout << "leg  " << i << "  " << _data->_legController->data[i].p << std::endl;
      }

      _data->_legController->updateCommand();
      counter++;

      rate.sleep();
  }
}