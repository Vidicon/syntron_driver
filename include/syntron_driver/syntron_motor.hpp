#pragma once
#include <string>
#include <cstdint>
#include "syntron_driver/syntron_can_com.hpp"

class SyntronMotor
{
    SyntronMotor(const std::string & interface, uint8_t id);
    ~SyntronMotor() = default;
private:
    std::shared_ptr<SyntronCanCom> device_;
    std::string interface_name_;
    uint8_t can_id_;
};