#include "move_master/diffbot_state.h"
#include "move_master/diffbot_state_machine.h"

#include "ros/console.h"

namespace move_master {


DiffbotStateBase::DiffbotStateBase(DiffbotState::StateEnum state, std::shared_ptr<DiffbotStateMachineInterface> machine)
  : state_(state),
    machine_(machine)
{
}

void DiffbotStateBase::addTransition(uint cmd, DiffbotState::StateEnum next_state)
{
  trans_map_[cmd] = next_state;
}

DiffbotState::StateEnum DiffbotStateBase::DetermineNextState(uint cmd)
{
  if (trans_map_.find(cmd) == trans_map_.end()) {
    return DiffbotState::kNone;
  }

  return trans_map_[cmd];
}
  
DiffbotState::StateEnum DiffbotStateBase::getState()
{
  return state_;
}

void DiffbotStateBase::transition(DiffbotState::StateEnum next_state, const diffbot_msgs::MoveCmd::ConstPtr& for_cmd)
{
  if (!machine_) {
    ROS_ERROR_NAMED("DiffbotState", "machine is NULL");
    return;
  }

  machine_->requestTransition(next_state, for_cmd);
}

void DiffbotStateBase::processCmd(const diffbot_msgs::MoveCmd::ConstPtr& cmd)
{
  ROS_INFO("[DiffbotStateBase][%d]processCmd %d:%s", state_, cmd->cmd, cmd->param.c_str());
  DiffbotState::StateEnum next_state = DetermineNextState(cmd->cmd);
  if (next_state != DiffbotState::kNone) {
    transition(next_state, cmd);
  }
}

} // namespace move_master