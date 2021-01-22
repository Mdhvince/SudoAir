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

    
public:
    PID_Controller(const float min_motor_thrust, const float max_motor_thrust, float drone_mass);
    PID_Controller();
    PID_Controller(const PID_Controller &source);
    ~PID_Controller();


    // From Position controller to motors
    float thrust(float z, float z_des, float vel_z_des, float vel_z, float ff_z_ddot);

    // from position controller to attitude controller
    float phi_cmd(float y, float y_des, float vel_y_des, float vel_y, float ff_y_ddot);

    // from attitude controller to motors
    float turning_moment(float phi, float phi_cmd, float phi_cmd_dot, float phi_dot_sensor);


    
};
#endif
