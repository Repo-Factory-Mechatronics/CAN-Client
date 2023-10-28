#ifndef CAN_CLIENT_H
#define CAN_CLIENT_H

#include <robot_libraries/custom_ros_types.hpp>
#include <robot_libraries/custom_data_types.hpp>
#include <robot_libraries/public_service_apis.hpp>
#include <robot_libraries/can_commands.hpp>

namespace CanClient
{
    int sendNothing(const RTL::node_t& node, const RTL::send_frame_client_t& can_client);
    int setBotInSafeMode(const RTL::node_t& node, const RTL::send_frame_client_t& can_client);
    int turnOnLight(const RTL::node_t& node, const RTL::send_frame_client_t& can_client);
    int turnOffLight(const RTL::node_t& node, const RTL::send_frame_client_t& can_client);
    int killRobot(const RTL::node_t& node, const RTL::send_frame_client_t& can_client);
    int allClear(const RTL::node_t& node, const RTL::send_frame_client_t& can_client);
    int make_motor_request 
    (
        const RTL::node_t& node, 
        const RTL::send_frame_client_t& can_client, 
        const std::vector<float>& thrusts
    );
}

#endif