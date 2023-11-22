#pragma once
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"


using namespace std;

class RB3_cpp_publisher : public rclcpp::Node{
public:
  // Create publisher for publishing velocity commands to topic "cmd_vel"
  RB3_cpp_publisher()
    : Node("rb3_cpp_publisher")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);
  }


  // Class function to be called when you want to publish velocity commands
  void publish_vel(float lin_vel_cmd, float ang_vel_cmd){
    // Set angular velocity to desired value (ie. turning)
    msg.angular.x = 0;
    msg.angular.y = 0;
    msg.angular.z = ang_vel_cmd;

    // Set linear velocity to desired value (ie. forward and backwards)
    msg.linear.x = lin_vel_cmd;
    msg.linear.y = 0;
    msg.linear.z = 0;

    RCLCPP_INFO(this->get_logger(), "Publishing: %f , %f", this->msg.linear.x, this->msg.angular.z);

    publisher_->publish(msg);
  }
private:
  // Private variables used for the publisher
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  geometry_msgs::msg::Twist msg;
};

