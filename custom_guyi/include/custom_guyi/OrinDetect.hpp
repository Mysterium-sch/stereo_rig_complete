#ifndef ORIN_DETECT_HPP
#define ORIN_DETECT_HPP

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cv_bridge/cv_bridge.h>
#include "std_msgs/msg/string.hpp"

class OrinDetect : public rclcpp::Node
{
public:
    OrinDetect();

private:
    void timer_callback();
    std::string jetsonCheck(const std::string &device);

    std::string device;
    std::string orin;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    float depth;
};

#endif // ORIN_DETECT_HPP
