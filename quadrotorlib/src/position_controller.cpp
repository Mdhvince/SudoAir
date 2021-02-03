#include "quadrotorlib/position_controller.h"
#include <algorithm>
#include <cmath>

PositionController::PositionController(){ }
PositionController::~PositionController(){ }


std::array <std::array<float, 3>, 3> PositionController::Rot_mat(float phi, float theta, float psi){
    
    // transform vector of body fixed frame to Inertial frame
    std::array <std::array<float, 3>, 3> R {{
        {{ cosf(psi)*cosf(theta)-sinf(phi)*sinf(psi)*sinf(theta), -cosf(phi)*sinf(psi), cosf(psi)*sinf(theta)+cosf(theta)*sinf(phi)*sinf(psi) }},
        {{ cosf(theta)*sinf(psi)+cosf(psi)*sinf(phi)*sinf(theta), cosf(phi)*cosf(psi), sinf(psi)*sinf(theta)-cosf(theta)*sinf(phi)*cosf(psi) }},
        {{ -cosf(phi)*sinf(theta), sinf(phi), cosf(phi)*cosf(theta) }}
    }};
    return R;
}

float PositionController::desired_acc(std::string axis, float kp, float kd,
                                       std::unordered_map<std::string, float> &state,
                                       std::unordered_map<std::string, float> &state_des, float &ff_term){
    std::string vel {};
    std::string acc {};
    if(axis == "z") {vel = "z_dot"; acc = "z_ddot";}
    else if (axis == "y") {vel = "y_dot"; acc = "y_ddot";}
    else if (axis == "x") {vel = "x_dot"; acc = "x_ddot";}
    else {;}

    float err {state_des.at(axis) - state.at(axis)};
    float err_dot {state_des.at(vel) - state.at(vel)};
    float acc_des = ff_term + kd * err_dot + kp * err;

    return acc_des;
}

std::array<float, 3> PositionController::R_b3(float phi, float theta, float psi){

    std::array<float, 3> rb3 = {
        cosf(psi)*sinf(theta) + cosf(theta)*sinf(phi)*sinf(psi),
        sinf(psi)*sinf(theta) - cosf(psi)*cosf(theta)*sinf(phi),
        cosf(phi)*cosf(theta)
    };
    return rb3;
}


float PositionController::thrust_cmd(std::array<float, 3> &kp_pos, std::array<float, 3> &kd_pos,
                          std::unordered_map<std::string, float> &state, std::unordered_map<std::string, float> &state_des,
                          float m, float g, const float min_F, const float max_F, float &z_ff){
    
    // position controller Z
    float z_ddot_des = this->desired_acc("z", kp_pos.at(2), kd_pos.at(2), state, state_des, z_ff);
    float F_hover = m * (z_ddot_des + g);

    // float F = F_hover * R_b3(phi, theta, psi)[2]; // 3rd component of R_b3 = R * b3 = R * [0, 0, 1]^T
    float F = F_hover;

    F = std::clamp(F, min_F, max_F);
    state_des.at("z_ddot") = (F / m) - g;

    return F;
}


void PositionController::angle_cmd(std::array<float, 3> &kp_pos, std::array<float, 3> &kd_pos,
                          std::unordered_map<std::string, float> &state, std::unordered_map<std::string, float> &state_des,
                          float m, float g, const float min_F, const float max_F, float &x_ff, float &y_ff){
    

    float max_acc = (max_F / m) - g;
    float min_acc = (min_F / m) - g;
    float psi = state.at("psi");

    // position controller X Y
    float x_ddot_des = this->desired_acc("x", kp_pos.at(0), kd_pos.at(0), state, state_des, x_ff);
    float y_ddot_des = this->desired_acc("y", kp_pos.at(1), kd_pos.at(1), state, state_des, y_ff);

    x_ddot_des = std::clamp(x_ddot_des, min_acc, max_acc);
    y_ddot_des = std::clamp(y_ddot_des, min_acc, max_acc);

    float phi_c = (1 / g) * x_ddot_des * sinf(psi) - y_ddot_des * cosf(psi);
    float theta_c = (1 / g) * x_ddot_des * cosf(psi) + y_ddot_des * sinf(psi);
    float psi_c = psi;

    // updates
    state_des.at("x_ddot") = x_ddot_des;
    state_des.at("y_ddot") = y_ddot_des;
    state_des.at("phi") = phi_c;
    state_des.at("theta") = theta_c;
    state_des.at("psi") = psi_c;
}