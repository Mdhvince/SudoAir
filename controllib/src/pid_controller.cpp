#include <algorithm>
#include <cmath>
#include "controllib/pid_controller.h"

PID_Controller::PID_Controller(const float max_motor_thrust, float drone_mass)
    :max_motor_thrust{max_motor_thrust},
    drone_mass{drone_mass},
    gravity{9.81},
    I_xx{0.018} {
    std::cout<< "Initializing PID Controller\n";
}

// PID_Controller::PID_Controller()
//     :PID_Controller{0.0, 100} {}

// PID_Controller::PID_Controller(const PID_Controller &source)
//     :PID_Controller{source.max_motor_thrust, source.drone_mass} {
//     std::cout<< "Copy of PID Made\n";
// }

PID_Controller::~PID_Controller(){ }



float PID_Controller::phi_cmd(float y_sensor, float y_des, float y_dot_des, float y_dot_sensor, float ff_y_ddot){
    float error {y_des - y_sensor};
    float error_dot {y_dot_des - y_dot_sensor};
    float k_p {10.0};
    float k_d {30.0};

    float phi_c = (-1 / this->gravity) * (ff_y_ddot + k_p * error + k_d * error_dot);

    return phi_c;
}
//update ff_y_ddot

float PID_Controller::turning_moment(float phi_sensor, float phi_cmd, float phi_cmd_dot, float phi_dot_sensor){
    float error {phi_cmd - phi_sensor};
    float error_dot {phi_cmd_dot - phi_dot_sensor};
    float k_p {10.0};
    float k_d {30.0};
    
    float phi_cmd_ddot {0.0};
    float u2 = this->I_xx * (phi_cmd_ddot + k_p * error + k_d * error_dot);

    return u2;
}

float PID_Controller::thrust(float z_sensor, float z_des, float z_dot_des, float z_dot_sensor, float ff_z_ddot){
    float error {z_des - z_sensor};
    float error_dot {z_dot_des - z_dot_sensor};
    float k_p {10.0};
    float k_d {30.0};

    float u_bar = (ff_z_ddot + k_p * error + k_d * error_dot + this->gravity) * this->drone_mass;
    float u = std::min(u_bar, this->max_motor_thrust);

    if(u < 0.0)
        u = 0.0;

    return u;
}
//update ff_z_ddot

