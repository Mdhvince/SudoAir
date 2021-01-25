#ifndef _PID_CONTROLLER_H_
#define _PID_CONTROLLER_H_
#include <iostream>
#include <array>

class PID_Controller{

protected:
    float i_error_z;
    float i_error_y;

    
public:
    PID_Controller();
    PID_Controller(const PID_Controller &source);
    ~PID_Controller();


    // From Position controller to motors
    // angle_wf = phi theta psi in world frame coordinates
    float control_altitude(std::array<float, 9> &xyz_state, std::array<float, 9> &xyz_state_des, std::array<float, 9> &angle_state_wf, const float gravity, float drone_mass, const float min_motor_thrust, const float max_motor_thrust);

    // from position controller to attitude controller
    float control_lateral(std::array<float, 9> &xyz_state, std::array<float, 9> &xyz_state_des, float u, float drone_mass);

    // from attitude controller to motors
    float control_attitude(std::array<float, 9> &angle_state_wf, std::array<float, 9> &angle_state_wf_des, float I_xx);

    
};
#endif
