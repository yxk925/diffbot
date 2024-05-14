#include <ros/ros.h>

#include "config/diffbot_base_config.h"
#include "base_controller/base_controller.h"
#include "motor_controllers/lx98n/lx98n_controller.h"
#include "motor_controllers/lx98n/lx98n_motor_driver.h"


using namespace diffbot;

class BaseControllerNode {

public:
    ros::NodeHandle nh;

    //bool imu_is_initialized;

    Lx98nMotorController motor_controller_right;
    Lx98nMotorController motor_controller_left;

    BaseController<Lx98nMotorController, Lx98nMotorDriver> base_controller;

    BaseControllerNode()
    : motor_controller_left(MOTOR_LEFT)
    , motor_controller_right(MOTOR_RIGHT)
    , base_controller(nh, &motor_controller_left, &motor_controller_right)
     {

    }

    void setup()
    {
        base_controller.setup();
        base_controller.init();

        ROS_INFO("Initialize DiffBot Motor Controllers");
        motor_controller_left.begin();
        motor_controller_right.begin();
        ROS_INFO("Setup finished");
    }


    void loop()
    {
        // The main control loop for the base_conroller.
        // This block drives the robot based on a defined control rate
        ros::Duration command_dt = ros::Time::now() - base_controller.lastUpdateTime().control;
        if (command_dt.toSec() >= ros::Duration(1.0 / base_controller.publishRate().control_, 0).toSec())
        {
            base_controller.read();
            base_controller.write();
            base_controller.lastUpdateTime().control = ros::Time::now();
        }

        // This block stops the motor when no wheel command is received
        // from the high level hardware_interface::RobotHW
        command_dt = ros::Time::now() - base_controller.lastUpdateTime().command_received;
        if (command_dt.toSec() >= ros::Duration(E_STOP_COMMAND_RECEIVED_DURATION, 0).toSec())
        {
            ROS_FATAL("Emergency STOP");
            base_controller.eStop();
        }

        // This block publishes the IMU data based on a defined imu rate
        ros::Duration imu_dt = ros::Time::now() - base_controller.lastUpdateTime().imu;
        if (imu_dt.toSec() >= base_controller.publishRate().period().imu_)
        {
            // Sanity check if the IMU is connected
            /*
            if (!imu_is_initialized)
            {
                //imu_is_initialized = initIMU();
                if(imu_is_initialized)
                    ROS_INFO("IMU Initialized");
                else
                    ROS_FATAL("IMU failed to initialize. Check your IMU connection.");
            }
            else
            {
                //publishIMU();
            }
            base_controller.lastUpdateTime().imu = ros::Time::now();
            */
        }

        // This block displays the encoder readings. change DEBUG to 0 if you don't want to display
        if(base_controller.debug())
        {
            ros::Duration debug_dt = ros::Time::now() - base_controller.lastUpdateTime().debug;
            if (debug_dt.toSec() >= base_controller.publishRate().period().debug_)
            {
                base_controller.printDebug();
                base_controller.lastUpdateTime().debug = ros::Time::now();
            }
        }
    }
};


int main(int argc, char **argv)
{
  ros::init(argc, argv, "base_controller_node");
  
  BaseControllerNode node;
  
  node.setup();

  ros::Rate loop_rate(10);
  
  while (ros::ok())
  {
    node.loop();

    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}