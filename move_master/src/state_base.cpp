#include "move_master/diffbot_state.h"

namespace move_master {


DiffbotStateBase::DiffbotStateBase(DiffbotState::StateEnum state, std::shared_ptr<DiffbotStateMachinInterface> machine)
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

} // namespace move_master