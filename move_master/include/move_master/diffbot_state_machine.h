#pragma once

#include "move_master/diffbot_state.h"

namespace move_master {

class DiffbotStateMachineInterface : public std::enable_shared_from_this<DiffbotStateMachineInterface>
{
  public:
    virtual void requestTransition(DiffbotState::StateEnum next_state,
      const diffbot_msgs::MoveCmd::ConstPtr& for_cmd) = 0;
    virtual void processCmd(const diffbot_msgs::MoveCmd::ConstPtr& cmd) = 0;
};

class DiffbotStateMachine :
  public DiffbotStateMachineInterface
  
{
public:
  DiffbotStateMachine();
  void init();

  // implement DiffbotStateMachineInterface
  void requestTransition(DiffbotState::StateEnum next_state,
    const diffbot_msgs::MoveCmd::ConstPtr& for_cmd) override;
  void processCmd(const diffbot_msgs::MoveCmd::ConstPtr& cmd) override;
private:
  DiffbotState::StateEnum getCurState();
  std::shared_ptr<DiffbotStateBase> createState(DiffbotState::StateEnum state);
  void enterState(DiffbotState::StateEnum state);
  void leaveState(DiffbotState::StateEnum state);
  
  std::shared_ptr<DiffbotStateBase>  cur_state_;
  std::list<std::shared_ptr<DiffbotStateBase>>  garbage_states_;
};

} // namespace move_master