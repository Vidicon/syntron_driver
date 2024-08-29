#include "syntron_driver/syntron_can_com.hpp"


SyntronCanCom::SyntronCanCom(const std::string& interface_name, const uint8_t can_id)
    : interface_name_(interface_name),
    device_id_(can_id),
    host_id_(can_id + 0xA0),
    socket_can_(std::make_shared<SocketCAN>(interface_name))
{

    socket_can_->registerCallback(std::bind(&SyntronCanCom::canRxCallback, this, std::placeholders::_1));

    timeout_thread_ = std::thread(&SyntronCanCom::timeoutChecker, this);

}

SyntronCanCom::~SyntronCanCom()
{
    running_ = false;
    if (timeout_thread_.joinable()) {
        timeout_thread_.join();
    }
}

// WriteRegister(WriteRegType::PARAM, 0x0000, 0, WriteMethod::NORMAL);
bool SyntronCanCom::writeRegister(WriteRegType regType, uint16_t address, uint16_t data)
{
    return writeRegister(device_id_, regType, address, data, WriteMethod::NORMAL);
}

void SyntronCanCom::timeoutChecker()
{
    while(running_)
    {
        auto now = std::chrono::steady_clock::now();
        for(auto it = pending_request_queue_.begin(); it != pending_request_queue_.end();)
        {
            if(std::chrono::duration_cast<std::chrono::milliseconds>(now - it->time_stamp).count() > 1000)
            {
                std::cerr << "Timeout waiting for reply" << std::endl;

                ReplyResponse response;
                response.device_id = it->device_id;
                response.function_code = it->function_code;
                response.address = it->address;
                response.response_type = ResponseType::TIMEOUT;

                it->promise->set_value(response);
                it = pending_request_queue_.erase(it);
            }
            else
            {
                ++it;
            }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
}

bool SyntronCanCom::writeRegister(uint8_t id, WriteRegType regType, uint16_t address, uint16_t data, WriteMethod method)
{
    struct can_frame frame;
    CanFuncionCode function_code;
    bool is_broadcast = (method == WriteMethod::BROADCAST);

    function_code = writeFCTable[static_cast<int>(regType)][static_cast<int>(method)];
    frame.can_id = (uint16_t) id + (!is_broadcast << 8);
    frame.data[0] = host_id_;
    frame.data[1] = static_cast<uint8_t>(function_code);
    frame.data[2] = static_cast<uint8_t>(address >> 8);
    frame.data[3] = static_cast<uint8_t>(address & 0xFF);
    frame.data[4] = static_cast<uint8_t>(data >> 8);
    frame.data[5] = static_cast<uint8_t>(data & 0xFF);
    
    frame.len = 6;
    ReplyRequest rr;
    rr.promise = std::make_shared<std::promise<ReplyResponse>>();
    if(method == WriteMethod::NORMAL)
    {

        
        rr.device_id = id;
        rr.function_code = static_cast<uint8_t>(function_code);
        rr.address = address;
        rr.time_stamp = std::chrono::steady_clock::now();
        pending_request_queue_.push_back(rr);
    }
    // std::cout << "Sending frame with function code: 0x" << std::hex << (int) function_code << std::dec << std::endl;
    socket_can_->transmit(frame);

    if(method != WriteMethod::NORMAL)
    {
        return true;
    }

    std::future<ReplyResponse> future = rr.promise->get_future();

    ReplyResponse responce = future.get();

    // if(responce.response_type == ResponseType::NORMAL)
    // {
    //     std::cout << "Received a normal reply" << std::endl;
    // }
    // else if(responce.response_type == ResponseType::ERROR)
    // {
    //     std::cerr << "Received an error reply" << std::endl;
    // }
    // else if(responce.response_type == ResponseType::TIMEOUT)
    // {
    //     std::cerr << "Received a timeout reply" << std::endl;
    // }
    return true;
}

bool SyntronCanCom::readRegister(uint8_t id, ReadRegType regType, uint16_t address, uint16_t & data, ReadMethod method)
{
    struct can_frame frame;
    CanFuncionCode function_code;

    function_code = readFCTable[static_cast<int>(regType)][static_cast<int>(method)];
    frame.can_id = (uint16_t) id + (1 << 8); // PTP bit set
    frame.data[0] = host_id_;
    frame.data[1] = static_cast<uint8_t>(function_code);
    frame.data[2] = static_cast<uint8_t>(address >> 8);
    frame.data[3] = static_cast<uint8_t>(address & 0xFF);
    
    frame.len = 4;
    ReplyRequest rr;
    rr.promise = std::make_shared<std::promise<ReplyResponse>>();
    rr.device_id = id;
    rr.function_code = static_cast<uint8_t>(function_code);
    rr.address = address;
    rr.time_stamp = std::chrono::steady_clock::now();
    pending_request_queue_.push_back(rr);
    
    // std::cout << "Sending frame with function code: 0x" << std::hex << (int) function_code << std::dec << std::endl;
    socket_can_->transmit(frame);

    std::future<ReplyResponse> future = rr.promise->get_future();

    ReplyResponse responce = future.get();

    if(responce.response_type == ResponseType::NORMAL)
    {
        data = responce.data[0];
        return true;
    }
    else if(responce.response_type == ResponseType::ERROR)
    {
        std::cerr << "Received an error reply" << std::endl;
        return false;
    }
    else if(responce.response_type == ResponseType::TIMEOUT)
    {
        std::cerr << "Received a timeout reply" << std::endl;
        return false;
    }

    return true;
}

void SyntronCanCom::logErrorReply(uint8_t device, uint8_t function_code, uint8_t error_code, uint16_t address)
{
 std::cerr << "Error reply: device: " << (int) device \
 << " function code: " << (int) function_code \
 << " address: 0x" << std::hex << (int) address << std::dec;

 switch(error_code)
 {
    case 0x01:
        std::cerr << " (DLC illegal)";
        break;
    case 0x02:
        std::cerr << " (Register address illegal)";
        break;
    case 0x03:
        std::cerr << " (Register value out of range)";
        break;
    case 0x04:
        std::cerr << " (Register read only)";
        break;
 }
 std::cerr << std::endl;
}

CanFuncionCodeType SyntronCanCom::idToType(uint8_t id)
{
    return static_cast<CanFuncionCodeType>((id / 10));
}

void SyntronCanCom::checkMultiMaster(const struct can_frame & frame) // check if someone else sends a frame to my device
{
    uint8_t target_id = frame.can_id & 0x0FF;
    uint8_t body_id = frame.data[0];
    if( target_id == device_id_)
    {
        std::cerr << "Other master with id: 0x" << std::hex << (int) body_id << std::dec << " detected sending data to my device" << std::endl;
    }
}

void SyntronCanCom::matchReplyRequest(uint8_t device_id, uint8_t host_id, uint8_t function_code, uint16_t address, uint16_t data) 
{
    for(auto it = pending_request_queue_.begin(); it != pending_request_queue_.end();)
    {
        if(it->device_id == device_id)
        {
            if(it->address == address)
            {
                // std::cerr << "match found for reply" << std::endl;
                ReplyResponse response;
                response.device_id = device_id;
                response.function_code = function_code;
                response.address = address;
                response.data[0] = data;
                response.response_type = ResponseType::NORMAL;

                it->promise->set_value(response);
                it = pending_request_queue_.erase(it);
            }
            else
            {
                ++it;
            }
        }
        else
        {
            ++it;
        }
    }
}

void SyntronCanCom::processReadParam(const struct can_frame & frame)
{
    uint8_t head_id = frame.can_id & 0x0FF;
    uint8_t body_id = frame.data[0];
    uint8_t function_code = frame.data[1];
    uint8_t fc_offset = function_code - static_cast<int>(CanFuncionCode::READ_SINGLE_PARAMETER);
    uint16_t address = frame.data[2] << 8 | frame.data[3];
    // std::cout << "Received a read param frame, fc: "<< (int) function_code << " fc_off: " << (int) fc_offset << std::endl;
    switch(static_cast<ReplyOffset>(fc_offset))
    {
        case ReplyOffset::REQUEST:
        case ReplyOffset::REQUEST_DUAL:
        case ReplyOffset::REQUEST_NOSAVE:
            checkMultiMaster(frame);
            break;

        case ReplyOffset::REPLY_ERROR:
        case ReplyOffset::REPLY_DUAL_ERROR:
        case ReplyOffset::REPLY_NOSAVE_ERROR:
        {
            uint16_t errorCode = frame.data[4] << 8 | frame.data[5];
            logErrorReply(body_id, function_code, errorCode, address);
            break;
        }
        case ReplyOffset::REPLY_NORMAL:
            if(head_id == host_id_ && body_id == device_id_)
            {
                uint16_t data = frame.data[4] << 8 | frame.data[5];
                // std::cout << "Received param reply, register: 0x" << std::hex << (int) address << " data: 0x" << (int) data << std::dec << std::endl;
                matchReplyRequest(body_id, head_id, 0, address, data); 
            }
            break;
        case ReplyOffset::REPLY_DUAL_NORMAL:
            break;
        case ReplyOffset::REPLY_NOSAVE_NORMAL:
            break;
        
            break;
        default:
            std::cerr << "Received a CAN frame with unknown function code offset" << std::endl;
            break;
    }
}


void SyntronCanCom::processWriteParam(const struct can_frame & frame)
{
    uint8_t head_id = frame.can_id & 0x0FF;
    uint8_t body_id = frame.data[0];
    uint8_t function_code = frame.data[1];
    uint8_t fc_offset = function_code - static_cast<int>(CanFuncionCode::WRITE_SINGLE_PARAMETER);
    uint16_t address = frame.data[2] << 8 | frame.data[3];
    // std::cout << "Received a write param frame, fc: "<< (int) function_code << " fc_off: " << (int) fc_offset << std::endl;
    switch(static_cast<ReplyOffset>(fc_offset))
    {
        case ReplyOffset::REQUEST:
        case ReplyOffset::REQUEST_DUAL:
        case ReplyOffset::REQUEST_NOSAVE:
            checkMultiMaster(frame);
            break;

        case ReplyOffset::REPLY_ERROR:
        case ReplyOffset::REPLY_DUAL_ERROR:
        case ReplyOffset::REPLY_NOSAVE_ERROR:
        {
            uint16_t errorCode = frame.data[4] << 8 | frame.data[5];
            logErrorReply(body_id, function_code, errorCode, address);
            break;
        }
        case ReplyOffset::REPLY_NORMAL:
            if(head_id == host_id_ && body_id == device_id_)
            {
                uint16_t data = frame.data[4] << 8 | frame.data[5];
                // std::cout << "Received param reply, register: 0x" << std::hex << (int) address << " data: 0x" << (int) data << std::dec << std::endl;
                matchReplyRequest(body_id, head_id, 0, address); 
            }
            break;
        case ReplyOffset::REPLY_DUAL_NORMAL:
            break;
        case ReplyOffset::REPLY_NOSAVE_NORMAL:
            break;
        
            break;
        default:
            std::cerr << "Received a CAN frame with unknown function code offset" << std::endl;
            break;
    }
}

void SyntronCanCom::processReadStatus(const struct can_frame & frame)
{
    uint8_t head_id = frame.can_id & 0x0FF;
    uint8_t body_id = frame.data[0];
    uint8_t function_code = frame.data[1];
    uint8_t fc_offset = function_code - static_cast<int>(CanFuncionCode::READ_SINGLE_STATUS);
    uint16_t address = frame.data[2] << 8 | frame.data[3];
    switch(static_cast<ReplyOffset>(fc_offset))
    {
        case ReplyOffset::REQUEST:
        case ReplyOffset::REQUEST_DUAL:
        case ReplyOffset::REQUEST_NOSAVE:
            checkMultiMaster(frame);
            break;

        case ReplyOffset::REPLY_ERROR:
        case ReplyOffset::REPLY_DUAL_ERROR:
        case ReplyOffset::REPLY_NOSAVE_ERROR:
        {
            uint16_t errorCode = frame.data[4] << 8 | frame.data[5];
            logErrorReply(body_id, function_code, errorCode, address);
            break;
        }
        case ReplyOffset::REPLY_NORMAL:
            if(head_id == host_id_ && body_id == device_id_)
            {
                uint16_t data = frame.data[4] << 8 | frame.data[5];
                // std::cout << "Received status register: 0x" << std::hex << (int) address << " data: 0x" << (int) data << std::dec << std::endl;
            }
            break;
        case ReplyOffset::REPLY_DUAL_NORMAL:
            break;
        case ReplyOffset::REPLY_NOSAVE_NORMAL:
            break;
        
            break;
        default:
            std::cerr << "Received a CAN frame with unknown function code offset" << std::endl;
            break;
    }
}

void SyntronCanCom::canRxCallback(const struct can_frame & frame)
{
    uint8_t head_id = frame.can_id & 0x0FF; // bit 0-7 (8 bits)
    bool ptp = (frame.can_id >> 8) & 0x01; // bit 8 (1 bit)
    uint8_t priority = (frame.can_id >> 9) & 0x03; // bit 9-10 (2 bits)
    if(frame.len < 2)
    {
        std::cerr << "Received a CAN frame with less than 2 bytes" << std::endl;
        return;
    }
    
    uint8_t body_id = frame.data[0];
    uint8_t function_code = frame.data[1];
    CanFuncionCodeType type = idToType(function_code);
    // if(type != CanFuncionCodeType::REPORT_STATUS)
    // {
    //     std::cout << std::hex <<  "Received a head_id: 0x" << (int) head_id << " body_id: 0x" << (int) body_id << " function_code: 0x" << (int) function_code << " ptp: 0x" << (int) ptp << " priority: 0x" << (int) priority <<  " " << std::dec << std::endl;
    // }
    switch(type)
    {   
        case CanFuncionCodeType::UNKNOWN_FUNCTION_CODE:
            if(head_id == host_id_)
            {
                std::cerr << "Device does not know function code" << std::endl;
                return;
            }
            std::cerr << "Received a CAN frame with unknown function code" << std::endl;
            break;
        case CanFuncionCodeType::READ_STATUS:
            processReadStatus(frame);
            break;
        case CanFuncionCodeType::READ_PARAM:
            processReadParam(frame);
            break;
        case CanFuncionCodeType::WRITE_PARAM:
            processWriteParam(frame);
            break;
        case CanFuncionCodeType::WRITE_COMMAND:
            break;
        case CanFuncionCodeType::WRITE_SPECIAL:
            break;
        case CanFuncionCodeType::WRITE_REPLYLESS:
            checkMultiMaster(frame);
            break;
        case CanFuncionCodeType::WRITE_SPECIAL_REPLYLESS:
            checkMultiMaster(frame);
            break;
        case CanFuncionCodeType::WRITE_PARAM_BROADCAST:
            break;
        case CanFuncionCodeType::WRITE_COMMAND_BROADCAST:
            break;
        case CanFuncionCodeType::WRITE_SPECIAL_BROADCAST:
            break;
        case CanFuncionCodeType::REPORT_STATUS:
            break;
        case CanFuncionCodeType::REPORT_FAULT:
            break;
        default:
            std::cerr << "Received a CAN frame with unknown function code" << std::endl;
            break;
    }

    // if(function_code )
    // uint8_t device_id =
    // uint8_t master_id = 

    // std::cout << std::hex <<  "Received a head_id: 0x" << (int) head_id << " body_id: 0x" << (int) body_id << " function_code: 0x" << (int) function_code << " ptp: 0x" << (int) ptp << " priority: 0x" << (int) priority <<  " " << std::dec << std::endl;

    // if(ptp && head_id != host_id_ || head_id != device_id_)
    // {
    //     // std::cerr << "Received a CAN frame with PTP bit set and head_id not matching my ids" << std::endl;
    //     return;
    // }

    // switch (function_code)
    // {
    // case CanFuncionCode::ERROR_REPLY_ILLEGAL_FUNCTION_REQUEST:
    //     std::cerr << "ERROR_REPLY_ILLEGAL_FUNCTION_REQUEST" << std::endl;
    //     break;
    
    
    // default:
    //     break;
    // }

    // std::cout << "Received a CAN frame: " << std::hex << (int) frame.can_id << std::dec << " " << std::endl;

    // if (msg->id == can_id_)
    // {
    //     // process the message
    // }
}

bool SyntronCanCom::connect()
{
    is_connected_ = socket_can_->connect();
    return is_connected_; 
}

void SyntronCanCom::disconnect()
{
    std::cout << "disconnecting" << std::endl;
    socket_can_->disconnect();
    is_connected_ = false;
}

// bool SyntronCanCom::writeRegister(RegisterType type, uint16_t address, uint16_t data)
// {
//     // can_msgs::msg::Frame frame;
//     // frame.id = can_id_;
//     // frame.dlc = 8;
//     // frame.is_extended = false;
//     // frame.is_rtr = false;
//     // frame.data[0] = static_cast<uint8_t>(type);
//     // frame.data[1] = static_cast<uint8_t>(address >> 8);
//     // frame.data[2] = static_cast<uint8_t>(address & 0xFF);
//     // frame.data[3] = static_cast<uint8_t>(data >> 8);
//     // frame.data[4] = static_cast<uint8_t>(data & 0xFF);
//     // socket_can_.send(frame);
// }

// bool SyntronCanCom::readRegister(uint16_t address)
// {
//     // can_msgs::msg::Frame frame;
//     // frame.id = can_id_;
//     // frame.dlc = 3;
//     // frame.is_extended = false;
//     // frame.is_rtr = false;
//     // frame.data[0] = static_cast<uint8_t>(RegisterType::PARAM);
//     // frame.data[1] = static_cast<uint8_t>(address >> 8);
//     // frame.data[2] = static_cast<uint8_t>(address & 0xFF);
//     // socket_can_.send(frame);
// }