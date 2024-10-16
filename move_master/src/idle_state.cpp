#include "move_master/diffbot_state.h"

#include "ros/console.h"

namespace move_master {

IdleState::IdleState(std::shared_ptr<DiffbotStateMachineInterface> machine)
  : DiffbotStateBase(DiffbotState::kIdle, machine)
{
  addTransition(diffbot_msgs::MoveCmd::kForward, DiffbotState::kSimpleMoving);
  addTransition(diffbot_msgs::MoveCmd::kBackward, DiffbotState::kSimpleMoving);
  addTransition(diffbot_msgs::MoveCmd::kTurnLeft, DiffbotState::kSimpleMoving);
  addTransition(diffbot_msgs::MoveCmd::kTurnRight, DiffbotState::kSimpleMoving);
  addTransition(diffbot_msgs::MoveCmd::kFollowme, DiffbotState::kFollowing);
}

void IdleState::enter()
{
  ROS_INFO_NAMED("IdleState", "enter Idle state");
}

void IdleState::leave()
{
  ROS_INFO_NAMED("IdleState", "leave Idle state");
}
void IdleState::processCmd(const diffbot_msgs::MoveCmd::ConstPtr& cmd)
{
  ROS_INFO("[IdleState]processCmd %d:%s", cmd->cmd, cmd->param.c_str());
  DiffbotStateBase::processCmd(cmd);
}

} // namespace move_master