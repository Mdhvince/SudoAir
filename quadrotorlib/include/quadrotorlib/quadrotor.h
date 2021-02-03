#ifndef _QUADROTOR_H_
#define _QUADROTOR_H_
#include <iostream>
#include <array>

#include "position_controller.h"


class QuadRotor{
    PositionController pos_ctrl;
    std::unordered_map<std::string, float> quad_specs;

private:

    QuadRotor();
    QuadRotor(std::unordered_map<std::string, float> quad_specs, std::unordered_map<std::string, float> state);
    QuadRotor(const QuadRotor &source);
    ~QuadRotor();

    std::pair<float, float> get_next_waypoint();
    std::pair<float, float> get_home_waypoint();
    bool is_waypoint_reached(std::pair<float, float> &point);

    void start_rotors();
    void stop_rotors();

    void control_position();
    void send_cmd();
    void sense();


       /*
    auto sudoAir = make_unique<QuadRotor>(m, inertia, max_motor_thrust_N, min_motor_thrust_N, xyz_state);
    sudoAir->start_rotors();

    while(true){
        next_wp = get_next_waypoint();

        if(next_wp == home_wp && waypoint_reached(home_wp)){
            sudoAir->stop_rotors();
            break;
        }

        while(!waypoint_reached(next_wp)){
            sudoAir->control_position();
            sudoAir->send_cmd()
        }
    }

    */

};

#endif