#include "move_master/diffbot_state.h"

#include "ros/console.h"

namespace move_master {

SimpleMovingState::SimpleMovingState(std::shared_ptr<DiffbotStateMachineInterface> machine)
  : DiffbotStateBase(DiffbotState::kSimpleMoving, machine)
{
  addTransition(diffbot_msgs::MoveCmd::kStop, DiffbotState::kIdle);
}

void SimpleMovingState::enter()
{
  ROS_INFO_NAMED("SimpleMovingState", "enter SimpleMoving state");
}

void SimpleMovingState::leave()
{
  ROS_INFO("SimpleMovingState", "leave SimpleMoving state");
}
void SimpleMovingState::processCmd(const diffbot_msgs::MoveCmd::ConstPtr& cmd)
{
  ROS_INFO("[SimpleMovingState]processCmd %d:%s", cmd->cmd, cmd->param.c_str());
  try {
    float distance = 0.0f;
    float angle = 0.0f;
    switch(cmd->cmd) {
      case diffbot_msgs::MoveCmd::kForward:
        distance = std::stof(cmd->param);
        move_agent_.moveForward(distance);
        break;
      case diffbot_msgs::MoveCmd::kBackward:
        distance = std::stof(cmd->param);
        move_agent_.moveBackward(distance);
        break;
      case diffbot_msgs::MoveCmd::kTurnLeft:
        if (move_agent_.getState() != MoveActionAgent::kLeftTurning) {
          angle = std::stof(cmd->param);
          move_agent_.turnLeft(angle);
        }
        break;
      case diffbot_msgs::MoveCmd::kTurnRight:
        if (move_agent_.getState() != MoveActionAgent::kLeftTurning) {
          angle = std::stof(cmd->param);
          move_agent_.turnRight(angle);
        }
        break;
      case diffbot_msgs::MoveCmd::kStop:
        move_agent_.stop();
        break;
    }
    
    DiffbotStateBase::processCmd(cmd);
  } catch(std::exception& e) {
    ROS_ERROR_NAMED("SimpleMovingState", "[processCmd]excpation:%s", e.what());
  }
}

} // namespace move_master