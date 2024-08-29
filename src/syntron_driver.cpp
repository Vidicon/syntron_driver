#include "syntron_driver/syntron_driver.hpp"

SyntronDriver::SyntronDriver() : Node("syntron_driver")
{
  RCLCPP_INFO(get_logger(), "Starting syntron_driver_node");
  device_ = std::move(std::make_shared<SyntronCanCom>("can0", 1));
  device_->connect();
  // device_->writeRegister(1, WriteRegType::PARAM, 0x10, 1, WriteMethod::NORMAL);
  
  for(int i = 0; i < 100; i++)
  {
    uint16_t data;
    device_->readRegister(1, ReadRegType::PARAM, i, data, ReadMethod::NORMAL);
    std::cout << i <<" Data: " << data << std::endl;
    // std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
  
  // motor_ = std::make_shared<SyntronMotor>(interface_name_, can_id_);
}