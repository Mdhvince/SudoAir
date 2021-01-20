#include <algorithm>
#include <cmath>
#include "controllib/pid_controller.h"

PID_Controller::PID_Controller(const float max_motor_thrust, float drone_mass)
    :max_motor_thrust{max_motor_thrust}, drone_mass{drone_mass}, i_err_z{0.0}, feedforward_acc_z{0.0}, gravity{9.81} {
    std::cout<< "Initializing PID Controller\n";
}

PID_Controller::PID_Controller()
    :PID_Controller{0.0, 100} {}

PID_Controller::PID_Controller(const PID_Controller &source)
    :PID_Controller{source.max_motor_thrust, source.drone_mass} {
    std::cout<< "Copy of PID Made\n";
}

PID_Controller::~PID_Controller(){ }

float PID_Controller::proportional_gain(float t_term, float damping_ratio){ return (1 / pow(t_term, 2)) * (1 + 2 * damping_ratio); }
float PID_Controller::derivative_gain(float t_term, float damping_ratio){ return (1 / t_term) * (1 + 2 * damping_ratio); }
float PID_Controller::integral_gain(float t_term){ return 1 / pow(t_term, 3); }

float PID_Controller::altitude_control(float z, float z_des, float vel_z_des, float vel_z, float dt){
    float error {z_des - z};

    float error_dot {vel_z_des - vel_z};
    this->i_err_z += error * dt;

    float k_p {10.0};
    float k_d {30.0};
    float k_i {0.0};

    float a_bar = this->feedforward_acc_z + k_p * error + k_d * error_dot + k_i * this->i_err_z; // output acc after the controller
    float a_max = (this->max_motor_thrust - (this->drone_mass * this->gravity)) / this->drone_mass; // max acc possible by the drone
    float acc = std::min(a_bar, a_max);
    float hover = -this->drone_mass * this->gravity + (this->drone_mass * this->gravity);

    if(acc < 0)
        acc = hover - vel_z_des; // so we can desselerate but not too fast

    this->feedforward_acc_z = acc;
    float thrust = this->drone_mass * (acc + this->gravity);
    
    return thrust;

}