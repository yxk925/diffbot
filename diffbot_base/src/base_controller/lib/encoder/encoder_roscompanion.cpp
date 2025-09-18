#include "encoder_roscompanion.h"

#include <std_msgs/Int32MultiArray.h>

namespace roscompanion
{

std::string kEncoderTopName = "/encoders_array";
// 定义全局变量来存储接收到的数据
std::vector<int32_t> position_data;
ros::Subscriber encoder_sub;
std::vector<int32_t> pre_received_data;

// 定义回调函数：当收到新消息时自动执行
void encoderArrayCallback(const std_msgs::Int32MultiArray& msg) {
  for (int i = 0; i < msg.data.size(); ++i) {
    int32_t delta = 0;
    if (i < pre_received_data.size()) {
      int32_t pre_value = pre_received_data[i];
      delta = msg.data[i] - pre_value;
      // 处理编码器值溢出问题
      if (delta > 2147483648LL) {        // 2^31 = 2147483648
        delta -= 4294967296LL;         // 2^32 = 4294967296
      } else if (delta < -2147483648LL) {
        delta += 4294967296LL;
      }
    }

    if (i >= position_data.size()) {
      position_data.push_back(delta);
    } else {
      position_data[i] += delta;
    }
  }

  pre_received_data = msg.data;
}

// 创建订阅者，指定话题名和回调函数
  
EncoderAdpter::EncoderAdpter(ros::NodeHandle& nh, unsigned char channel)
  : channel_(channel)
{
    // Initialize the subscriber only once
    if (encoder_sub.getTopic() == "") {
        encoder_sub = nh.subscribe(kEncoderTopName, 10, encoderArrayCallback);
    }
}

int EncoderAdpter::read()
{
  if (channel_ < position_data.size()) {
      return position_data[channel_];
  }
  
  return 0;
}
void EncoderAdpter::write(int32_t p)
{
  if (channel_ < position_data.size()) {
      position_data[channel_] = p;
  }
}
    

    

} // namespace wiringpi_encoder