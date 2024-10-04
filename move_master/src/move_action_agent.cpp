#include "move_master/move_action_agent.h"
#include "ros/console.h"

namespace move_master {

MoveActionAgent::MoveActionAgent()
{

}

bool MoveActionAgent::moveForward(float distance)
{
  ROS_INFO("[MoveActionAgent]moveForward, distance:%f", distance);
  geometry_msgs::Pose pose;
  pose.position.x = distance;
  pose.orientation.w = 1.0;

  return move(pose);
}

bool MoveActionAgent::moveBackward(float distance)
{
  ROS_INFO("[MoveActionAgent]moveBackward, distance:%f", distance);
  geometry_msgs::Pose pose;
  pose.position.x = -distance;
  pose.orientation.w = 1.0;

  return move(pose);
}

bool MoveActionAgent::turnLeft(float angle)
{
  ROS_INFO("[MoveActionAgent]turnLeft, angle:%f", angle);
  geometry_msgs::Pose pose;
  pose.orientation.z = angle;

  pose.orientation.w = 1.0;

  return move(pose);
}

bool MoveActionAgent::turnRight(float angle)
{
  ROS_INFO("[MoveActionAgent]turnRight, angle:%f", angle);
  geometry_msgs::Pose pose;
  pose.orientation.z = -angle;

  pose.orientation.w = 1.0;

  return move(pose);
}

bool MoveActionAgent::move(const geometry_msgs::Pose& pose)
{
  MoveBaseClient ac("move_base", true);
  const int32_t kWaitNSec = 300*1000;
  if (!ac.waitForServer(ros::Duration(100, kWaitNSec))) {
    ROS_ERROR_NAMED("MoveActionAgent", "Action server NOT connected!");
    return false;
  }
  move_base_msgs::MoveBaseGoal goal;
  
  //we'll send a goal to the robot to move 1 meter forward
  goal.target_pose.header.frame_id = "base_link";
  goal.target_pose.header.stamp = ros::Time::now();
  
  goal.target_pose.pose = pose;
  
  ROS_INFO_NAMED("MoveActionAgent", "Sending goal");
  ac.sendGoal(goal,
              boost::bind(&MoveActionAgent::doneCb, this, _1, _2),
              MoveBaseClient::SimpleActiveCallback(), 
              MoveBaseClient::SimpleFeedbackCallback());

  return true;
}

void MoveActionAgent::doneCb(const actionlib::SimpleClientGoalState& state,
                const move_base_msgs::MoveBaseResultConstPtr& result)
{
  ROS_INFO_NAMED("MoveActionAgent", "doneCb, Finished in state [%s]", state.toString().c_str());
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