#ifndef CAN_CLIENT_H
#define CAN_CLIENT_H

#include <robot_libraries/custom_ros_types.hpp>
#include <robot_libraries/custom_data_types.hpp>
#include <robot_libraries/public_service_apis.hpp>
#include <robot_libraries/can_commands.hpp>

namespace CanClient
{
    void sendNothing(const RTL::node_t& node, const RTL::send_frame_client_t& can_client);
    void setBotInSafeMode(const RTL::node_t& node, const RTL::send_frame_client_t& can_client);
    void turnOnLight(const RTL::node_t& node, const RTL::send_frame_client_t& can_client);
    void turnOffLight(const RTL::node_t& node, const RTL::send_frame_client_t& can_client);
    void killRobot(const RTL::node_t& node, const RTL::send_frame_client_t& can_client);
    void allClear(const RTL::node_t& node, const RTL::send_frame_client_t& can_client);
    void make_motor_request 
    (
        const RTL::node_t& node, 
        const RTL::send_frame_client_t& can_client, 
        const std::vector<float>& thrusts
    );
}

#endif