#include "can_client.hpp"
#include <cstdint>
#include <array>

constexpr const int MAX_POWER                            = 100;
constexpr const std::array<uint8_t, 5> TURN_OFF_LIGHT    {0x04, 0x00, 0x04, 0x00, 0x00};
constexpr const std::array<uint8_t, 5> ENABLE_LIGHT      {0x04, 0x00, 0x04, 0x00, 0x01};
constexpr const std::array<uint8_t, 5> TURN_ON_LIGHT     {0x04, 0x00, 0x04, 0x00, 0x64};
constexpr const std::array<uint8_t, 5> SAFE_MODE         {0x00, 0x00, 0x00, 0x00, 0x04};
constexpr const std::array<uint8_t, 3> ALL_CLEAR         {0x00A, 0, 0};
constexpr const std::array<uint8_t, 3> KILL              {0x000, 0, 0};
constexpr const std::array<uint8_t, 1> NOTHING           {0x65};       // Sending number above 100 (in hex) will have no affect on system

void CanClient::sendNothing(const RTL::node_t& node, const RTL::send_frame_client_t& can_client)
{
    const auto can_frame = 
    RTL::SendFrameRequest
    {
        .can_id = static_cast<uint8_t>(CanDriver::Command::MOTOR),
        .can_dlc = 1,
        .can_data = NOTHING.data()
    };
    ServiceAPIs::SendFrame(node, can_client, can_frame);
}

void CanClient::setBotInSafeMode(const RTL::node_t& node, const RTL::send_frame_client_t& can_client)
{
    const auto can_frame = 
    RTL::SendFrameRequest
    {
        .can_id = static_cast<uint8_t>(CanDriver::Command::STOW),
        .can_dlc = 5,
        .can_data = SAFE_MODE.data()
    };
    ServiceAPIs::SendFrame(node, can_client, can_frame);
}

void CanClient::turnOnLight(const RTL::node_t& node, const RTL::send_frame_client_t& can_client)
{
    const auto enable_frame = 
    RTL::SendFrameRequest
    {
        .can_id = static_cast<uint8_t>(CanDriver::Command::STOW),
        .can_dlc = 5,
        .can_data = ENABLE_LIGHT.data()
    };
    ServiceAPIs::SendFrame(node, can_client, enable_frame);

    const auto turn_on_frame = 
    RTL::SendFrameRequest
    {
        .can_id = static_cast<uint8_t>(CanDriver::Command::STOW),
        .can_dlc = 5,
        .can_data = TURN_ON_LIGHT.data()
    };
    ServiceAPIs::SendFrame(node, can_client, turn_on_frame);
}

void CanClient::turnOffLight(const RTL::node_t& node, const RTL::send_frame_client_t& can_client)
{
    const auto turn_on_frame = 
    RTL::SendFrameRequest
    {
        .can_id = static_cast<uint8_t>(CanDriver::Command::STOW),
        .can_dlc = 5,
        .can_data = TURN_OFF_LIGHT.data()
    };
    ServiceAPIs::SendFrame(node, can_client, turn_on_frame);
}

void CanClient::killRobot(const RTL::node_t& node, const RTL::send_frame_client_t& can_client)
{
    const auto turn_on_frame = 
    RTL::SendFrameRequest
    {
        .can_id = static_cast<uint8_t>(CanDriver::Command::SOFTKILL),
        .can_dlc = 0,
        .can_data = KILL.data()
    };
    ServiceAPIs::SendFrame(node, can_client, turn_on_frame);
}

void CanClient::allClear(const RTL::node_t& node, const RTL::send_frame_client_t& can_client)
{
    const auto turn_on_frame = 
    RTL::SendFrameRequest
    {
        .can_id = static_cast<uint8_t>(CanDriver::Command::CLEARERR),
        .can_dlc = 0,
        .can_data = ALL_CLEAR.data()
    };
    ServiceAPIs::SendFrame(node, can_client, turn_on_frame);
}

void CanClient::make_motor_request 
(
    const RTL::node_t& node, 
    const RTL::send_frame_client_t& can_client, 
    const std::vector<float>& thrusts
)
{
    /* Thrusts come out of PID as a float between -1 and 1; motors need int value from -100 to 100 */
    std::vector<int> convertedThrusts;
    for (const float thrust : thrusts)
    {
        convertedThrusts.push_back(((int)(thrust * MAX_POWER)));
    }

    /* 
    * We have integer values that are 32 bits (4 bytes) but need values of one byte to send to motor
    * We can extract using an and mask and get last 8 bits which in hex is 0xFF. Char size is one byte
    * which is why we use an array of chars
    */          

    std::vector<unsigned char> byteThrusts;
    for (const int thrust : convertedThrusts)
    {
        byteThrusts.push_back(thrust & 0xFF);
    }
    /* See exactly our 8 thrust values sent to motors */

    ////////////////////////////////////////// BUILD REQUEST //////////////////////////////////////////
    /* 
    * Our frame will send CAN request 
    * one byte for each value -100 to 100 
    */
   const auto can_frame =
    RTL::SendFrameRequest
    {
        .can_id = static_cast<uint8_t>(CanDriver::Command::MOTOR),
        .can_dlc = (int8_t)byteThrusts.size(),
        .can_data = byteThrusts.data()
    };

    ServiceAPIs::SendFrame(node, can_client, can_frame);
}
