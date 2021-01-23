#include <algorithm>
#include <cmath>
#include "controllib/pid_controller.h"

PID_Controller::PID_Controller(const float min_motor_thrust, const float max_motor_thrust, float drone_mass)
    :min_motor_thrust{min_motor_thrust}, max_motor_thrust{max_motor_thrust},
    drone_mass{drone_mass},
    gravity{9.81},
    I_xx{0.1},
    i_error_z{0.0}, i_error_y{0.0} {
    std::cout<< "Initializing PID Controller\n";
}

// PID_Controller::PID_Controller()
//     :PID_Controller{0.0, 100} {}

// PID_Controller::PID_Controller(const PID_Controller &source)
//     :PID_Controller{source.max_motor_thrust, source.drone_mass} {
//     std::cout<< "Copy of PID Made\n";
// }

PID_Controller::~PID_Controller(){ }


// {z, z_dot, z_ddot, z_des, z_dot_des}, float phi
float PID_Controller::thrust(std::array<float, 5> &state_z, float phi){
    float error {state_z.at(3) - state_z.at(0)};
    float error_dot {state_z.at(4) - state_z.at(1)};
    this->i_error_z += error;
    
    float k_p {1.3};
    float k_d {2.};
    float k_i {0.0};

    float u_bar = state_z.at(2) + k_p * error + k_d * error_dot + k_i * this->i_error_z + this->gravity;

    float u = (u_bar / std::cos(phi)) * this->drone_mass;
    // float u = this->drone_mass * (this->gravity - u_bar) / std::cos(phi);

    u = std::min(u, this->max_motor_thrust);

    if(u < this->min_motor_thrust)
        u = this->min_motor_thrust;
    
    return u;
}


// Position controller : (Y-axis : Lateral)
float PID_Controller::phi_cmd(std::array<float, 5> &state_y, float u){
    float error {state_y.at(3) - state_y.at(0)};
    float error_dot {state_y.at(4) - state_y.at(1)};

    float k_p {100};
    float k_d {20.0};

    float y_ddot_cmd = state_y.at(2) + k_p * error + k_d * error_dot;


    float val = y_ddot_cmd * (this->drone_mass / -u) ;
    float boundary1 = std::max((float)-0.99, val);
    
    val = std::min((float)0.99, boundary1);
    float phi_c = asin(val);

    return phi_c;
}

// Attitude controller : (Orientation : roll/phi -  - ) -> ang and ang rate infos comes from IMU
//float phi, float phi_dot, float phi_ddot, float phi_cmd, float phi_cmd_dot
float PID_Controller::turning_moment(std::array<float, 5> &state_phi){
    float error {state_phi.at(3) - state_phi.at(0)};
    float error_dot {state_phi.at(4) - state_phi.at(1)};
    
    float k_p {180.0};
    float k_d {10.0};

    float u2 = this->I_xx * (state_phi.at(2) + k_p * error + k_d * error_dot);

    return u2;
}



