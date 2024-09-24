#include "move_master/diffbot_state.h"

#include "ros/console.h"

namespace move_master {

SimpleMovingState::SimpleMovingState(std::shared_ptr<DiffbotStateMachineInterface> machine)
  : DiffbotStateBase(DiffbotState::kIdle, machine)
{
  addTransition(diffbot_msgs::MoveCmd::kStop, DiffbotState::kIdle);
}

void SimpleMovingState::enter()
{
  ROS_INFO_NAMED("SimpleMovingState", "enter Idle state");
}

void SimpleMovingState::leave()
{
  ROS_INFO_NAMED("SimpleMovingState", "leave Idle state");
}
void SimpleMovingState::processCmd(const diffbot_msgs::MoveCmd& cmd)
{
  ROS_INFO_NAMED("SimpleMovingState", "processCmd %d:%s", cmd.cmd, cmd.param.c_str());
  DiffbotStateBase::processCmd(cmd);
}

} // namespace move_master