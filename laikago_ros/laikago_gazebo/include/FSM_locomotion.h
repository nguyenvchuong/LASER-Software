#ifndef FSM_LOCOMOTION_H
#define FSM_LOCOMOTION_H

#include "QPLocomotion.h"
#include "FSM.h"

class FSM_locomotion : public FSM_State {
  public:
    FSM_locomotion(ControlFSMData* _controlFSMData);

    void run();

  private:
    // keep track of control iterations
    int iters = 0;
    QPLocomotion qp_locomotion;

    // Parse contact specific control and run QP for leg controller
    void LocomotionControlStep();
};
#endif