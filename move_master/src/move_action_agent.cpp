#include "move_master/move_action_agent.h"
#include "ros/console.h"

namespace move_master {

MoveActionAgent::MoveActionAgent()
{

}

MoveActionAgent::State MoveActionAgent::getState()
{
  return state_;
}

bool MoveActionAgent::moveForward(float distance)
{
  ROS_INFO("[MoveActionAgent]moveForward, distance:%f", distance);
  geometry_msgs::Pose pose;
  pose.position.x = distance;
  pose.orientation.w = 1.0;

  if (!move(pose)) {
    ROS_ERROR("[MoveActionAgent][moveForward]move FAILED");
    return false;
  }

  state_ = kForwardMoving;

  return true;
}

bool MoveActionAgent::moveBackward(float distance)
{
  ROS_INFO("[MoveActionAgent]moveBackward, distance:%f", distance);
  geometry_msgs::Pose pose;
  pose.position.x = -distance;
  pose.orientation.w = 1.0;

  if (!move(pose)) {
    ROS_ERROR("[MoveActionAgent][moveBackward]move FAILED");
    return false;
  }

  state_ = kBackwardMoving;

  return true;
}

bool MoveActionAgent::turnLeft(float angle)
{
  ROS_INFO("[MoveActionAgent]turnLeft, angle:%f", angle);
  geometry_msgs::Pose pose;
  pose.orientation.z = angle;

  pose.orientation.w = 1.0;

  if (!move(pose)) {
    ROS_ERROR("[MoveActionAgent][turnLeft]move FAILED");
    return false;
  }

  state_ = kLeftTurning;

  return true;
}

bool MoveActionAgent::turnRight(float angle)
{
  ROS_INFO("[MoveActionAgent]turnRight, angle:%f", angle);
  geometry_msgs::Pose pose;
  pose.orientation.z = -angle;

  pose.orientation.w = 1.0;

  if (!move(pose)) {
    ROS_ERROR("[MoveActionAgent][turnRight]move FAILED");
    return false;
  }

  state_ = kRightTurning;

  return true;
}

bool MoveActionAgent::move(const geometry_msgs::Pose& pose)
{
  ROS_INFO("[MoveActionAgent][move]waiting server ...");
  MoveBaseClient ac("move_base", true);
  // wait for 1 sec
  if (!ac.waitForServer(ros::Duration(1.0))) {
    ROS_ERROR("[MoveActionAgent][move]Action server NOT connected!");
    return false;
  }
  move_base_msgs::MoveBaseGoal goal;
  
  //we'll send a goal to the robot to move, pose related by base_link
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  
  goal.target_pose.pose = pose;
  
  ROS_INFO("[MoveActionAgent][move]Sending goal");
  ac.sendGoal(goal,
              boost::bind(&MoveActionAgent::doneCb, this, _1, _2),
              MoveBaseClient::SimpleActiveCallback(), 
              MoveBaseClient::SimpleFeedbackCallback());

  return true;
}

bool MoveActionAgent::stop()
{
  MoveBaseClient ac("move_base", true);
  const int32_t kWaitNSec = 300*1000;
  if (!ac.waitForServer(ros::Duration(100, kWaitNSec))) {
    ROS_ERROR_NAMED("MoveActionAgent", "Action server NOT connected!");
    return false;
  }
  ac.cancelAllGoals();

  return true;
}

void MoveActionAgent::doneCb(const actionlib::SimpleClientGoalState& state,
                const move_base_msgs::MoveBaseResultConstPtr& result)
{
  ROS_INFO_NAMED("MoveActionAgent", "doneCb, Finished in state [%s]", state.toString().c_str());
  state_ = kStop;
  // ROS_INFO_NAMED("MoveActionAgent", "doneCb, Answer: %i", result->sequence.back());
}

bool MoveActionAgent::checkAndConnect()
{/*
  if (ac_.isServerConnected) {
    return;
  }

  ac_.connect */
  return false;
}

} // namespace move_master