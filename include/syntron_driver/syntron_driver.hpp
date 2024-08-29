#pragma once

#include "rclcpp/rclcpp.hpp"
#include "syntron_driver/syntron_motor.hpp"

class SyntronDriver : public rclcpp::Node
{

public:
  SyntronDriver();
  ~SyntronDriver() = default;

private:
   std::shared_ptr<SyntronMotor> motor_;
   std::shared_ptr<SyntronCanCom> device_;
  // std::string interface_name_;
  // uint8_t can_id_;
};
