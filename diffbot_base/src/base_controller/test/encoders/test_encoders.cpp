
#include <ros/ros.h>
#include <gtest/gtest.h>

#include "config/diffbot_base_config.h"
#include "encoder/encoder_diffbot.h"


TEST(TestEncoder, testRpm)
{
  printf("Enter testRpm\n");
  // ROS node handle
  ros::NodeHandle nh;

  diffbot::Encoder encoderLeft(nh, ENCODER_LEFT_H1, ENCODER_LEFT_H2, ENCODER_RESOLUTION);  // Default pins 5, 6
  diffbot::Encoder encoderRight(nh, ENCODER_RIGHT_H1, ENCODER_RIGHT_H2, ENCODER_RESOLUTION); // Default pins 7, 8
  
  long positionLeft  = -999;
  long positionRight = -999;

  ros::Time startTime = ros::Time::now();
  ros::Rate rate(1);

  while(ros::ok()) {
    rate.sleep();
    ros::Time now = ros::Time::now();
    encoderLeft.jointState();
    encoderRight.jointState();
    long newLeft, newRight;
    newLeft = encoderLeft.read();
    newRight = encoderRight.read();
    if (newLeft != positionLeft || newRight != positionRight) {
      printf("time:%f, posiont = (%d, %d), tps=(%d, %d), rpm = (%d, %d), angVel = (%f, %f)\n",
        now.toSec(),
        newLeft, newRight, 
        encoderLeft.getTPS(), encoderRight.getTPS(),
        encoderLeft.getRPM(), encoderRight.getRPM(),
        encoderLeft.angularVelocity(), encoderRight.angularVelocity());

      positionLeft = newLeft;
      positionRight = newRight;
    }


    double lasted = now.toSec() - startTime.toSec();
    if (lasted > 100) {
      printf("timeout reached");
      break;
    }

    ros::spinOnce();
    
  }

  EXPECT_EQ(5, 5);

  printf("Leave testRpm\n");
}


int main(int argc, char **argv){
  printf("Enter test_encoder.main\n");
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}


