#pragma once

#include "diffbot_msgs/MoveCmd.h"

#include <map>
#include <memory>

namespace move_master {

struct DiffbotState{
  enum StateEnum{
    kNone,
    kIdle,
    kSimpleMoving,
    kFollowing,
    kPerformance,
    kMultiPerformance
  };
};

class DiffbotStateMachineInterface;
class DiffbotStateBase : public std::enable_shared_from_this<DiffbotStateBase>
{
  public:
    DiffbotStateBase(DiffbotState::StateEnum state, std::shared_ptr<DiffbotStateMachineInterface> machine);
    void addTransition(uint cmd, DiffbotState::StateEnum next_state);
    virtual DiffbotState::StateEnum DetermineNextState(uint cmd);
    virtual void enter() = 0;
    virtual void leave() = 0;
    virtual void processCmd(const diffbot_msgs::MoveCmd& cmd);

    DiffbotState::StateEnum getState();
  protected:
    void transition(DiffbotState::StateEnum next_state, const diffbot_msgs::MoveCmd& for_cmd);
  private:
    DiffbotState::StateEnum state_;
    std::shared_ptr<DiffbotStateMachineInterface> machine_;
    std::map<uint, DiffbotState::StateEnum> trans_map_;
};


class IdleState : public DiffbotStateBase {
  public:
    IdleState(std::shared_ptr<DiffbotStateMachineInterface> machine);
    virtual void enter();
    virtual void leave();
    virtual void processCmd(const diffbot_msgs::MoveCmd& cmd);
};

class SimpleMovingState : public DiffbotStateBase {
  SimpleMovingState(std::shared_ptr<DiffbotStateMachineInterface> machine);
    virtual void enter();
    virtual void leave();
    virtual void processCmd(const diffbot_msgs::MoveCmd& cmd);
};

} // namespace move_master