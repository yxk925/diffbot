#pragma once

#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <actionlib_tutorials/FibonacciAction.h>

namespace move_master {

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class MoveActionAgent
{
  public:
    enum State{
      kStop,
      kForwardMoving,
      kBackwardMoving,
      kLeftTurning,
      kRightTurning
    };
    MoveActionAgent();
    bool moveForward(float distance);
    bool moveBackward(float distance);
    bool turnLeft(float angle);
    bool turnRight(float angle);
    bool move(const geometry_msgs::Pose& pose);
    bool stop();
    State getState();
  private:
    bool checkAndConnect();
    void doneCb(const actionlib::SimpleClientGoalState& state,
                // const ResultConstPtr& result);
                const move_base_msgs::MoveBaseResultConstPtr& result);
    State state_;
};

} // namespace move_master