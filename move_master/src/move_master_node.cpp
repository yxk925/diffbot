
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "diffbot_msgs/MoveCmd.h"
#include "move_master/diffbot_state_machine.h"

using namespace move_master;

std::shared_ptr<DiffbotStateMachine> g_state_machine = std::make_shared<DiffbotStateMachine>();

void cmdCallback(const diffbot_msgs::MoveCmd::ConstPtr& cmd)
{
  ROS_INFO("cmdCallback %d:%s", cmd->cmd, cmd->param.c_str());
  g_state_machine->processCmd(cmd);
}

int main(int agc, char** agv)
{  
  ros::init(agc, agv, "move_master");

  ros::NodeHandle nh;

  ros::Rate loop_rate(10);

  g_state_machine->init();

  ros::Subscriber sub = nh.subscribe("move_master/cmd", 1, cmdCallback);

  while(ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }
}