#pragma once

#include "diffbot_msgs/MoveCmd.h"
#include "move_master/move_action_agent.h"

#include <map>
#include <memory>

namespace move_master {

#define ADD_STATE(m, s) m[s]=#s

struct DiffbotState{
  enum StateEnum{
    kNone,
    kIdle,
    kSimpleMoving,
    kFollowing,
    kPerformance,
    kMultiPerformance
  };

  static std::string to_string(StateEnum state) {
    static std::map<int, std::string> s_m;
    if(s_m.size() == 0) {
      ADD_STATE(s_m, kNone);
      ADD_STATE(s_m, kIdle);
      ADD_STATE(s_m, kSimpleMoving);
      ADD_STATE(s_m, kFollowing);
      ADD_STATE(s_m, kPerformance);
      ADD_STATE(s_m, kMultiPerformance);
    }

    if (s_m.find(state) == s_m.end()) {
      return "Unknown";
    }

    return s_m[state];
  }
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
    virtual void processCmd(const diffbot_msgs::MoveCmd::ConstPtr& cmd);

    DiffbotState::StateEnum getState();
  protected:
    void transition(DiffbotState::StateEnum next_state, const diffbot_msgs::MoveCmd::ConstPtr& for_cmd);
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
    virtual void processCmd(const diffbot_msgs::MoveCmd::ConstPtr& cmd);
};

class SimpleMovingState : public DiffbotStateBase {
  public:
    SimpleMovingState(std::shared_ptr<DiffbotStateMachineInterface> machine);
    virtual void enter();
    virtual void leave();
    virtual void processCmd(const diffbot_msgs::MoveCmd::ConstPtr& cmd);

    MoveActionAgent move_agent_;
};

} // namespace move_master