#include "arm_controller/arm_api.h"

namespace arm_controller {

ArmApi::ArmApi() {
  arm_sdk_ = std::make_unique<UNITREE_ARM::unitreeArm>(false);
  arm_sdk_->sendRecvThread->start();
  arm_sdk_->backToStart();
  arm_sdk_->setFsm(UNITREE_ARM::ArmFSMState::PASSIVE);
  arm_sdk_->setFsm(UNITREE_ARM::ArmFSMState::LOWCMD);
  while (arm_sdk_->_ctrlComp->recvState.state !=
             UNITREE_ARM::ArmFSMState::LOWCMD &&
         (!shutdown_)) {
    std::cout << "Warning: Robotic arm is not set to a right working mode."
              << std::endl;
    arm_sdk_->setFsm(UNITREE_ARM::ArmFSMState::LOWCMD);
    std::this_thread::sleep_for(std::chrono::microseconds(10));
  }
  arm_sdk_->sendRecvThread->shutdown();
}
ArmApi::~ArmApi() {
  arm_sdk_->sendRecvThread->shutdown();
  shutdown_ = true;
}

void ArmApi::getState(ArmLowState &state) {
  state.endPosture = arm_sdk_->lowstate->endPosture;
  state.q = arm_sdk_->lowstate->q;
  state.dq = arm_sdk_->lowstate->dq;
  state.ddq = arm_sdk_->lowstate->ddq;
  state.tau = arm_sdk_->lowstate->tau;
  state.temperature = arm_sdk_->lowstate->temperature;
  state.errorstate = arm_sdk_->lowstate->errorstate;
  state.isMotorConnected = arm_sdk_->lowstate->isMotorConnected;
  state.arm_fsm_state = arm_sdk_->_ctrlComp->recvState.state;
  state.arm_connected =
      (state.arm_fsm_state != UNITREE_ARM::ArmFSMState::INVALID);
}

void ArmApi::setCmd(const ArmLowCmd &cmd) {
  arm_sdk_->lowcmd->q = cmd.q;
  arm_sdk_->lowcmd->dq = cmd.dq;
  arm_sdk_->lowcmd->tau = cmd.tau;
  arm_sdk_->lowcmd->kp = cmd.kp;
  arm_sdk_->lowcmd->kd = cmd.kd;
  arm_sdk_->lowcmd->twsit = cmd.twsit;
  arm_sdk_->lowcmd->posture = cmd.posture;
}

}  // namespace arm_controller
