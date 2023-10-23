#pragma once
#include <robot_libraries/robot_types.hpp>

class CanClient
{
    public:
        CanClient();
        void sendFrame(int32_t can_id, int8_t can_dlc, unsigned char can_data[]);
        void setBotInSafeMode();
        void turnOnLight();
        void turnOffLight();
        void killRobot();
        void allClear();
        void sendNothing();
        std::vector<int> make_motor_request(const std::vector<float>& thrusts, int motor_count, int max_power);
    private:
        RTL::node_t node;
        RTL::ros_sendframe_client_t can_client;
};
