#include <algorithm>
#include <cmath>
#include "quadrotorlib/pid_controller.h"

PID_Controller::PID_Controller()
    :i_error_z{0.0}, i_error_y{0.0} {
    std::cout<< "Initializing PID Controller\n";
}
PID_Controller::~PID_Controller(){ }

float PID_Controller::control_altitude(std::array<float, 9> &xyz_state, std::array<float, 9> &xyz_state_des, std::array<float, 9> &angle_state_wf, const float gravity, float drone_mass, const float min_motor_thrust, const float max_motor_thrust){
    float error {xyz_state_des.at(2) - xyz_state.at(2)};
    float error_dot {xyz_state_des.at(5) - xyz_state.at(5)};
    this->i_error_z += error;
    
    float k_p {1.};
    float k_d {6.};
    float k_i {0.0};

    float u_bar = (xyz_state.at(8) + k_p * error + k_d * error_dot + k_i * this->i_error_z + gravity) * drone_mass;
    float u = u_bar / std::cos(angle_state_wf.at(0));

    u = std::min(u, max_motor_thrust);

    if(u < min_motor_thrust)
        u = min_motor_thrust;
    
    return u;
}


float PID_Controller::control_lateral(std::array<float, 9> &xyz_state, std::array<float, 9> &xyz_state_des, float u, float drone_mass){
    float error {xyz_state_des.at(1) - xyz_state.at(1)};
    float error_dot {xyz_state_des.at(4) - xyz_state.at(4)};

    float k_p {20.0};
    float k_d {1.0};

    float y_ddot_cmd = xyz_state.at(7) + k_p * error + k_d * error_dot;
    float val = y_ddot_cmd * -drone_mass / u ;
    float boundary1 = std::max((float)-0.99, val);
    val = std::min((float)0.99, boundary1);
    float phi_c = asin(val);

    return phi_c * (180/M_PI); //convert to degree for world frame coord (only for simulation)
}

float PID_Controller::control_attitude(std::array<float, 9> &angle_state_wf, std::array<float, 9> &angle_state_wf_des, float I_xx){
    float error {angle_state_wf_des.at(0) - angle_state_wf.at(0)};
    float error_dot {angle_state_wf_des.at(3) - angle_state_wf.at(3)};
    
    float k_p {250.0};
    float k_d {1.};

    float u2 = I_xx * (angle_state_wf.at(6) + k_p * error + k_d * error_dot);

    return u2;
}



