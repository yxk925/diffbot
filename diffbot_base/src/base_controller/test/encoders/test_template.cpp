
#include <ros/ros.h>
#include <gtest/gtest.h>

TEST(TestSuite, testCase1)
{
    EXPECT_EQ(5, 5);
}


int main(int argc, char **argv){
    testing::InitGoogleTest(&argc, argv);
    ros::init(argc, argv, "tester");
    ros::NodeHandle nh;
    return RUN_ALL_TESTS();
}