
#include "ros/ros.h"
#include "std_msgs/String.h"

int main(int agc, char** agv)
{  
  ros::init(agc, agv, "move_master");

  ros::NodeHandle nh;

  ros::Rate loop_rate(10);

  while(ros::ok())
  {
    ros::spinOnce();

    loop_rate.sleep();
  }
}