#include "syntron_driver/syntron_motor.hpp"

SyntronMotor::SyntronMotor(const std::string& interface_name, const uint8_t can_id)
    : interface_name_(interface_name), can_id_(can_id)
{
    // socket_can_.registerCallback(std::bind(&SyntronMotor::canCallback, this, std::placeholders::_1));

}