#ifndef _POSITION_CONTROLLER_H_
#define _POSITION_CONTROLLER_H_
#include <iostream>
#include <array>
#include <unordered_map>


class PositionController{

public:
    PositionController();
    ~PositionController();

    void control_altitude(std::array<float, 3> &kp_pos, std::array<float, 3> &kd_pos,
                          std::unordered_map<std::string, float> &state, std::unordered_map<std::string, float> &state_des,
                          std::unordered_map<std::string, float> &inp_plant,
                          float drone_mass, float gravity, const float min_motor_thrust, const float max_motor_thrust);
    
    void control_lateral(std::array<float, 3> &kp_pos, std::array<float, 3> &kd_pos,
                         std::unordered_map<std::string, float> &state, std::unordered_map<std::string, float> &state_des,
                         float drone_mass, float gravity, const float min_motor_thrust, const float max_motor_thrust);
    
    void desired_acc(std::string axis, float kp, float kd, float min_acc, float max_acc, float min_vel, float max_vel,
                     std::unordered_map<std::string, float> &state,
                     std::unordered_map<std::string, float> &state_des);
    
};
#endif