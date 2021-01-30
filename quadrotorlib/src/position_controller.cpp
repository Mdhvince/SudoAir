#include "quadrotorlib/position_controller.h"
#include <algorithm>
#include <cmath>

PositionController::PositionController(){ }
PositionController::~PositionController(){ }


void PositionController::desired_acc(std::string axis, float kp, float kd, float min_acc, float max_acc, float min_vel, float max_vel,
                                      std::unordered_map<std::string, float> &state,
                                      std::unordered_map<std::string, float> &state_des){
    
    std::string vel {};
    std::string acc {};
    if(axis == "z") {vel = "z_dot"; acc = "z_ddot";}
    else if (axis == "y") {vel = "y_dot"; acc = "y_ddot";}
    else if (axis == "x") {vel = "x_dot"; acc = "x_ddot";}
    else {std::cerr<< "Expected axis x, y or z. Got " << axis << " instead."<<std::endl;}

    float err {state_des.at(axis) - state.at(axis)};
    // we can vary our desired velocity in fonction of position error OR keep a constant desired velocity
    float vel_des = state_des.at(vel) + kp * err;
    state_des.at(vel) = std::clamp(vel_des, min_vel, max_vel);
    float err_dot {state_des.at(vel) - state.at(vel)};

    float acc_des = state.at(acc) + kd * err_dot + kp * err;
    state_des.at(acc) = std::clamp(acc_des, min_acc, max_acc);
}

void PositionController::control_altitude(std::array<float, 3> &kp_pos, std::array<float, 3> &kd_pos,
                                      std::unordered_map<std::string, float> &state, std::unordered_map<std::string, float> &state_des,
                                      std::unordered_map<std::string, float> &inp_plant,
                                      float drone_mass, float gravity, const float min_motor_thrust, const float max_motor_thrust){
                                        
    float max_acc = ((max_motor_thrust - static_cast<float>(.01)) / drone_mass) - gravity;
    float min_acc = (min_motor_thrust / drone_mass) - gravity;
    float max_vel = .3;
    float min_vel = .0;

    desired_acc("z", kp_pos.at(2), kd_pos.at(2), min_acc, max_acc, min_vel, max_vel, state, state_des);
    inp_plant.at("u1") = drone_mass * (gravity + state_des.at("z_ddot"));
}

void PositionController::control_lateral(std::array<float, 3> &kp_pos, std::array<float, 3> &kd_pos,
                                         std::unordered_map<std::string, float> &state,
                                         std::unordered_map<std::string, float> &state_des,
                                         float drone_mass, float gravity, const float min_motor_thrust, const float max_motor_thrust){

    float max_acc = ((max_motor_thrust - static_cast<float>(.01)) / drone_mass) - gravity;
    float min_acc = (min_motor_thrust / drone_mass) - gravity;
    float max_vel = .3;
    float min_vel = .0;

    desired_acc("x", kp_pos.at(0), kd_pos.at(0), min_acc, max_acc, min_vel, max_vel, state, state_des);
    desired_acc("y", kp_pos.at(1), kd_pos.at(1), min_acc, max_acc, min_vel, max_vel, state, state_des);
    
    float psi {state_des.at("psi")};
    float x_ddot {state_des.at("x_ddot")};
    float y_ddot {state_des.at("y_ddot")};

    state_des.at("phi") = (1/gravity) * (x_ddot * sin(psi) - y_ddot * cos(psi));
    state_des.at("theta") = (1/gravity) * (x_ddot * cos(psi) + y_ddot * sin(psi));
    // yaw in not playing to follow a trajectory so the desired yaw is considered as ok


    /*desired angular velocity in the body fixed frame (p_des, q_des, r_des)
    when working near hover p_des and q_des are 0. Since yaw (psi) in not
    playing to follow the trajectory, we set r_des (yaw angular velocity in
    body-fixed frame = yaw_ddot_desired (yaw angular vel in the inertial frame)*/
    state_des.at("r") = state_des.at("psi_dot");

}
