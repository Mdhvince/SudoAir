#ifndef _PID_CONTROLLER_H_
#define _PID_CONTROLLER_H_
#include <iostream>

class PID_Controller{

protected:
    float i_err_z;
    float feedforward_acc_z;
    float gravity;
    float drone_mass;
    const float max_motor_thrust;

    
public:
    PID_Controller(const float max_motor_thrust, float drone_mass);
    PID_Controller();
    PID_Controller(const PID_Controller &source);
    ~PID_Controller();

    float proportional_gain(float t_term, float damping_ratio);  // will change in future version to be based on a reinforcement learning algorithm
    float derivative_gain(float t_term, float damping_ratio);    // will change in future version to be based on a reinforcement learning algorithm
    float integral_gain(float t_term);                           // will change in future version to be based on a reinforcement learning algorithm

    float altitude_control(float z, float z_des, float vel_z_des, float vel_z, float dt);

    float get_acceleration_z() { return this->feedforward_acc_z; }

    
};
#endif
