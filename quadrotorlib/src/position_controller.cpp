#include "quadrotorlib/position_controller.h"
#include <algorithm>
#include <cmath>

PositionController::PositionController(){ }
PositionController::~PositionController(){ }


void PositionController::desired_acc(std::string axis, float kp, float kd, float min_acc, float max_acc,
                                      std::unordered_map<std::string, float> &state,
                                      std::unordered_map<std::string, float> &state_des){
    
    std::string vel {};
    std::string acc {};
    if(axis == "z") {vel = "z_dot"; acc = "z_ddot";}
    else if (axis == "y") {vel = "y_dot"; acc = "y_ddot";}
    else if (axis == "x") {vel = "x_dot"; acc = "x_ddot";}
    else {std::cerr<< "Expected axis x, y or z. Got " << axis << " instead."<<std::endl;}

    float err {state_des.at(axis) - state.at(axis)};
    float err_dot {state_des.at(vel) - state.at(vel)};

    float acc_des = state.at(acc) + kd * err_dot + kp * err;
    state_des.at(acc) = std::clamp(acc_des, min_acc, max_acc);

}

std::array <std::array<float, 3>, 3> PositionController::Rot_mat(float phi, float theta, float psi){
    
    std::array <std::array<float, 3>, 3> R {{
        {{ cosf(psi)*cosf(theta)-sinf(phi)*sinf(psi)*sinf(theta), -cosf(phi)*sinf(psi), cosf(psi)*sinf(theta)+cosf(theta)*sinf(phi)*sinf(psi) }},
        {{ cosf(theta)*sinf(psi)+cosf(psi)*sinf(phi)*sinf(theta), cosf(phi)*cosf(psi), sinf(psi)*sinf(theta)-cosf(theta)*sinf(phi)*cosf(psi) }},
        {{ -cosf(phi)*sinf(theta), sinf(phi), cosf(phi)*cosf(theta) }}
    }};
    return R;
}

void PositionController::control_altitude(std::array<float, 3> &kp_pos, std::array<float, 3> &kd_pos,
                                      std::unordered_map<std::string, float> &state, std::unordered_map<std::string, float> &state_des,
                                      std::unordered_map<std::string, float> &inp_plant,
                                      float drone_mass, float gravity, const float min_motor_thrust, const float max_motor_thrust){
    
    float phi {state.at("phi")};
    float theta {state.at("theta")};
    float psi {state.at("psi")};
    float R_b3 = Rot_mat(phi, theta, psi)[2][2];

    float max_acc = ((max_motor_thrust - static_cast<float>(.01)) / drone_mass) - gravity;
    float min_acc = (min_motor_thrust / drone_mass) - gravity;

    desired_acc("z", kp_pos.at(2), kd_pos.at(2), min_acc, max_acc, state, state_des);
    
    state_des.at("z_ddot") /= R_b3; // projecttion of the acceleration vector along the z-body axis
    float u1 = drone_mass * (gravity + state_des.at("z_ddot"));
 
    inp_plant.at("u1") = std::clamp(u1, min_motor_thrust, max_motor_thrust);
}

void PositionController::control_lateral(std::array<float, 3> &kp_pos, std::array<float, 3> &kd_pos,
                                         std::unordered_map<std::string, float> &state,
                                         std::unordered_map<std::string, float> &state_des,
                                         std::unordered_map<std::string, float> &inp_plant,
                                         float drone_mass, float gravity, const float min_motor_thrust, const float max_motor_thrust){

    float max_acc = ((max_motor_thrust - static_cast<float>(.01)) / drone_mass) - gravity;
    float min_acc = (min_motor_thrust / drone_mass) - gravity;

    desired_acc("x", kp_pos.at(0), kd_pos.at(0), min_acc, max_acc, state, state_des);
    desired_acc("y", kp_pos.at(1), kd_pos.at(1), min_acc, max_acc, state, state_des);
    
    // normalize acceleration by the thrust vector
    state_des.at("x_ddot") /= inp_plant.at("u1");
    state_des.at("y_ddot") /= inp_plant.at("u1");

    float x_ddot {state_des.at("x_ddot")};
    float y_ddot {state_des.at("y_ddot")};
    float psi {state_des.at("psi")};

    // cmd
    state_des.at("phi") = (1/gravity) * (x_ddot * sinf(psi) - y_ddot * cosf(psi));
    state_des.at("theta") = (1/gravity) * (x_ddot * cosf(psi) + y_ddot * sinf(psi));

}
