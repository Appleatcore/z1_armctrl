#ifndef ARM_API_H_
#define ARM_API_H_

#include <unitree_arm_sdk/control/unitreeArm.h>
#include <unitree_arm_sdk/message/LowlevelCmd.h>
#include <unitree_arm_sdk/message/LowlevelState.h>
#include <unitree_arm_sdk/model/ArmModel.h>

namespace arm_controller {

struct ArmLowState : public UNITREE_ARM::LowlevelState {
  UNITREE_ARM::ArmFSMState arm_fsm_state{UNITREE_ARM::ArmFSMState::INVALID};
  bool arm_connected{false};
};

using ArmLowCmd = UNITREE_ARM::LowlevelCmd;

using ArmModel = UNITREE_ARM::ArmModel;

using ArmFsmState = UNITREE_ARM::ArmFSMState;

using Z1ArmModel = UNITREE_ARM::Z1Model;

class Rate {
 public:
  using clock = std::chrono::high_resolution_clock;
  using time_point = clock::time_point;
  using nanoseconds = std::chrono::nanoseconds;
  explicit Rate(int freq) : event_time_(clock::now()), cycle_(int(1e9) / freq) {};
  void sync() { event_time_ = clock::now(); };
  void sleep() {
    event_time_ += cycle_;
    std::this_thread::sleep_until(event_time_);
  };

 private:
  nanoseconds cycle_;
  time_point event_time_;
};

class ArmApi {
 public:
  ArmApi();
  ~ArmApi();
  void getState(ArmLowState& state);
  void setCmd(const ArmLowCmd& cmd);

  void setFsmState(const ArmFsmState& state) { arm_sdk_->setFsm(state); }
  ArmModel* getArmModel() { return arm_sdk_->_ctrlComp->armModel; }
  /**
   * @brief This function consume much time. When you call this function, you
   * shouldn't get thread lock.
   *
   */
  void sendRecv() { arm_sdk_->sendRecv(); };

 protected:
  std::unique_ptr<UNITREE_ARM::unitreeArm> arm_sdk_;
  bool shutdown_{false};
};

}  // namespace arm_controller

#endif  // ARM_API_H_
