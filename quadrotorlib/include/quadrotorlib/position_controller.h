#ifndef _POSITION_CONTROLLER_H_
#define _POSITION_CONTROLLER_H_
#include <iostream>
#include <array>


class PositionController{

public:
    PositionController();
    ~PositionController();

    void control_altitude(std::array<float, 3> &kp_pos, std::array<float, 3> &kd_pos,
                          std::array<float, 9> &xyz_state, std::array<float, 9> &xyz_state_des,
                          std::array<float, 4> &inp_plant,
                          std::array<float, 9> &angle_state_wf_des,
                          std::array<float, 3> &pqr_state_des,
                          float drone_mass, float gravity, const float min_motor_thrust, const float max_motor_thrust);
    
    void control_lateral(std::array<float, 9> &xyz_state_des, std::array<float, 9> &angle_state_wf_des,
                    std::array<float, 3> &pqr_state_des, float gravity);
    
};
#endif