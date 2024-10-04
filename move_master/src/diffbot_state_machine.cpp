
#include "move_master/diffbot_state_machine.h"

#include "ros/console.h"

namespace move_master {

DiffbotStateMachine::DiffbotStateMachine()
{
}

void DiffbotStateMachine::init()
{
  enterState(DiffbotState::kIdle);
}

// implement DiffbotStateMachineInterface
void DiffbotStateMachine::requestTransition(DiffbotState::StateEnum next_state,
  const diffbot_msgs::MoveCmd::ConstPtr& for_cmd)
{
  ROS_INFO_NAMED("", "[DiffbotStateMachine]requestTransition to state:%s",\
    DiffbotState::to_string(next_state).c_str());
  enterState(next_state);
  processCmd(for_cmd);
}
void DiffbotStateMachine::processCmd(const diffbot_msgs::MoveCmd::ConstPtr& cmd)
{
  if (cur_state_) {
    cur_state_->processCmd(cmd);
  }
}

DiffbotState::StateEnum DiffbotStateMachine::getCurState()
{
  if (cur_state_) {
    return cur_state_->getState();
  }

  return DiffbotState::kNone;
}

std::shared_ptr<DiffbotStateBase> DiffbotStateMachine::createState(DiffbotState::StateEnum state)
{
  switch(state) {
    case DiffbotState::kIdle:  return std::make_shared<IdleState>(shared_from_this());
    case DiffbotState::kSimpleMoving:  return std::make_shared<SimpleMovingState>(shared_from_this());
    // case DiffbotState::kFollowing:  return std::make_shared<Following>(shared_from_this());
    default: 
      ROS_ERROR_NAMED("DiffbotStateMachine", "UNKNOWN state:%d", state);
      return std::shared_ptr<DiffbotStateBase>();
  }
}

void DiffbotStateMachine::enterState(DiffbotState::StateEnum state)
{
  if (cur_state_) {
    if (cur_state_->getState() == state)
    {
      // Already in the state, do nothing
      ROS_WARN_NAMED("DiffbotStateMachine", "[enterState]Already in state:%d-%s", state, DiffbotState::to_string(state));
      return;
    }
  }

  std::shared_ptr<DiffbotStateBase> next_state = createState(state);

  if (!next_state) {
    ROS_ERROR_NAMED("DiffbotStateMachine", "[enterState]CAN NOT enter state:%d-%s", state, DiffbotState::to_string(state));
    return;
  }

  if (cur_state_) {
    cur_state_->leave();
    cur_state_ = std::shared_ptr<DiffbotStateBase>();
  }

  next_state->enter();
  cur_state_ = next_state;
}

void DiffbotStateMachine::leaveState(DiffbotState::StateEnum state)
{
  if (cur_state_ && cur_state_->getState() != state) {
    // Do nothing
    return;
  }

  enterState(DiffbotState::kIdle);
}

} // namespace move_master
