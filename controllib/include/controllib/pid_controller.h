#ifndef _PID_CONTROLLER_H_
#define _PID_CONTROLLER_H_
#include <iostream>
#include <array>

class PID_Controller{

protected:
    const float gravity;
    float drone_mass;
    const float max_motor_thrust;
    const float min_motor_thrust;
    float I_xx;
    float i_error_z;
    float i_error_y;

    
public:
    PID_Controller(const float min_motor_thrust, const float max_motor_thrust, float drone_mass);
    PID_Controller();
    PID_Controller(const PID_Controller &source);
    ~PID_Controller();


    // From Position controller to motors
    float thrust(std::array<float, 5> &state_z, float phi);

    // from position controller to attitude controller
    float phi_cmd(std::array<float, 5> &state_y, float u);

    // from attitude controller to motors
    float turning_moment(std::array<float, 5> &state_phi);

    float get_drone_inertia(){return this->I_xx;}


    
};
#endif
