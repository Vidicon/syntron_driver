#pragma once

#include <string>
#include <iostream>
#include <memory>
#include <vector>
#include <chrono>
#include <future>         // std::promise, std::future
#include "syntron_driver/socketcan.hpp"
#include "syntron_driver/syntron_can_definitions.hpp"

enum class ResponseType
{
    NORMAL,
    ERROR,
    TIMEOUT
};

struct ReplyResponse
{
    uint8_t device_id;
    uint8_t function_code;
    uint16_t address;
    uint16_t data[2];
    ResponseType response_type;
};

struct ReplyRequest
{
    uint8_t device_id;
    uint8_t function_code;
    uint16_t address;
    std::chrono::time_point<std::chrono::steady_clock> time_stamp;
    std::shared_ptr<std::promise<ReplyResponse>> promise;
};

class SyntronCanCom
{
public:
    SyntronCanCom(const std::string& interface_name, const uint8_t device_id);
    ~SyntronCanCom();

    void setMasterID(uint8_t id) { host_id_ = id; }

    bool connect();
    void disconnect();
    
    bool isConnected() const { return is_connected_; }

    bool writeRegister(WriteRegType regType, uint16_t address, uint16_t data);
    bool writeRegister(uint8_t id, WriteRegType regType, uint16_t address, uint16_t data, WriteMethod method);

    bool readRegister(uint8_t id, ReadRegType regType, uint16_t address, uint16_t & data, ReadMethod method);

private:
    void canRxCallback(const struct can_frame & frame);
    CanFuncionCodeType idToType(uint8_t id);

    void timeoutChecker();
    void matchReplyRequest(uint8_t device_id, uint8_t host_id, uint8_t function_code, uint16_t address, uint16_t data = 0);
    void checkMultiMaster(const struct can_frame & frame);

    void processReadStatus(const struct can_frame & frame);
    void processReadParam(const struct can_frame & frame);
    void processWriteParam(const struct can_frame & frame);
    // void processWriteCommand(const struct can_frame & frame);
    // void processReportStatus(const struct can_frame & frame);
    // void processReportFault(const struct can_frame & frame);
    

    void logErrorReply(uint8_t device, uint8_t function_code, uint8_t error_code, uint16_t address);

    bool is_connected_ = false;
    std::string interface_name_;
    uint8_t device_id_;
    uint8_t host_id_;

    std::thread timeout_thread_;

    std::vector<ReplyRequest> pending_request_queue_;

    std::atomic<bool> running_ = true;

    std::shared_ptr<SocketCAN> socket_can_;

    const CanFuncionCode writeFCTable[3][3] = {
    {CanFuncionCode::WRITE_SINGLE_PARAMETER, CanFuncionCode::WRITE_SINGLE_PARAMETER_NO_REPLY, CanFuncionCode::BROADCAST_WRITE_SINGLE_PARAMETER},
    {CanFuncionCode::WRITE_SINGLE_VOLATILE_PARAMETER, CanFuncionCode::WRITE_SINGLE_VOLATILE_PARAMETER_NO_REPLY, CanFuncionCode::BROADCAST_WRITE_SINGLE_VOLATILE_PARAMETER},
    {CanFuncionCode::WRITE_SINGLE_COMMAND, CanFuncionCode::WRITE_SINGLE_COMMAND_NO_REPLY, CanFuncionCode::BROADCAST_WRITE_SINGLE_COMMAND}
    };

    const CanFuncionCode readFCTable[2][1] = {
    {CanFuncionCode::READ_SINGLE_STATUS},
    {CanFuncionCode::READ_SINGLE_PARAMETER}
    };

    
};