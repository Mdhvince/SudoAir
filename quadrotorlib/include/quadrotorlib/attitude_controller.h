#ifndef _ATTITUDE_CONTROLLER_H_
#define _ATTITUDE_CONTROLLER_H_
#include <iostream>
#include <array>

class AttitudeController{

public:
    AttitudeController();
    AttitudeController(const AttitudeController &source);
    ~AttitudeController();


    void control_attitude(std::array<float, 3> &kp_ang, std::array<float, 3> &kd_ang,
                          std::array<float, 3> &pqr_state, std::array<float, 3> &pqr_state_des,
                          std::array<float, 9> &angle_state_wf, std::array<float, 9> &angle_state_wf_des,
                          std::array<float, 4> &inp_plant, std::array<float, 3> &inertia);
    
    void apply_rotor_speed(std::array<float, 4> &inp_plant, float kf, float drone_mass, float gravity);
};
#endif
