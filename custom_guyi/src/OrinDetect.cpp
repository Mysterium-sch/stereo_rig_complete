#include "custom_guyi/OrinDetect.hpp"
#include <sensor_msgs/image_encodings.hpp>
#include <boost/filesystem.hpp>
#include <iostream>

std::string jetsonCheck(std::string device) {
        if(device == "jetson_1") {
      int x = system("ping -c1 -s1 192.168.0.100  > /dev/null 2>&1");
        if (x==0){
            return "Active";
        }else{
            return "Not Active";
        }
    } else if (device == "jetson_2"){
            int x = system("ping -c1 -s1 192.168.0.150  > /dev/null 2>&1");
        if (x==0){
            return "Active";
        }else{
            return "Not ACtive";
        }
    }
}

OrinDetect::OrinDetect()
  : rclcpp::Node("OrinDetect"), depth(0.0f)
{
    this->declare_parameter<std::string>("device", "");
    this->get_parameter("device", device);

    orin = "Not Active";
    publisher_ = this->create_publisher<std_msgs::msg::String>("orin", 10);
    timer_period = 0.5  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
}

    def timer_callback(self):
        orin = jetsonCheck(device)
        self.publisher_.publish(orin)
