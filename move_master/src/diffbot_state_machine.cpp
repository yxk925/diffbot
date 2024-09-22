
#include "move_master/diffbot_state_machine.h"

#include "ros/console.h"

namespace move_master {

DiffbotStateMachine::DiffbotStateMachine()
{
  enterState(DiffbotState::kIdle);
}

// implement DiffbotStateMachinInterface
void DiffbotStateMachine::requestTransition(DiffbotState::StateEnum next_state,
  const diffbot_msgs::MoveCmd& for_cmd)
{
  enterState(next_state);
  processCmd(for_cmd);
}
void DiffbotStateMachine::processCmd(const diffbot_msgs::MoveCmd& cmd)
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
    // case DiffbotState::kSimpleMoving:  return std::make_shared<SimpleMoving>(shared_from_this());
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
      // Do nothing
      return;
    }

    cur_state_->leave();
  }

  cur_state_ = createState(state);

  if (!cur_state_) {
    ROS_ERROR_NAMED("DiffbotStateMachine", "CAN NOT enter state:%d", state);
    return;
  }

  cur_state_->enter();
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
