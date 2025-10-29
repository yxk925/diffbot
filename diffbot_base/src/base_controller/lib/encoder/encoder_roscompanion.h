#pragma once

#include <ros/ros.h>

namespace roscompanion
{

class EncoderAdpter
{
  public:
    EncoderAdpter(ros::NodeHandle& nh, unsigned char channel);

    int32_t read();
    void write(int32_t p);
    static ros::Time last_update_time();
  private:
    const unsigned char channel_;
};

} // namespace wiringpi_encoder