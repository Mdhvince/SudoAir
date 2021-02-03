#include <algorithm>
#include <cmath>
#include "quadrotorlib/attitude_controller.h"

AttitudeController::AttitudeController(){ }
AttitudeController::~AttitudeController(){ }


void AttitudeController::control_attitude(std::array<float, 3> &kp_ang, std::array<float, 3> &kd_ang,
                                          std::unordered_map<std::string, float> &state, std::unordered_map<std::string, float> &state_des,
                                          std::unordered_map<std::string, float> &inp_plant, std::array<float, 3> &inertia){
    
    // att controller    
    float err_phi = state_des.at("phi") - state.at("phi");
    float err_theta = state_des.at("theta") - state.at("theta");
    float err_psi = state_des.at("psi") - state.at("psi");
    
    float err_p = state_des.at("p") - state.at("p");
    float err_q = state_des.at("q") - state.at("q");
    float err_r = state_des.at("r") - state.at("r");

    float M1 = inertia.at(0) * (kp_ang.at(0) * err_phi * kd_ang.at(0) * err_p);
    float M2 = inertia.at(1) * (kp_ang.at(1) * err_theta * kd_ang.at(1) * err_q);
    float M3 = inertia.at(2) * (kp_ang.at(2) * err_psi * kd_ang.at(2) * err_r);
}

// // thanks to equation (6) and (7) from : http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.169.1687&rep=rep1&type=pdf
// // the equation is established at near the hover state. That mean p = phi_dot, q = theta_dot & r = psi_dot
// void AttitudeController::apply_rotor_speed(std::unordered_map<std::string, float> &inp_plant, float kf, float drone_mass, float gravity){

//     float u1 = inp_plant.at("u1");   // [N]
//     float u2 = inp_plant.at("u2");   // [N m]
//     float u3 = inp_plant.at("u3");   // [N m]
//     float u4 = inp_plant.at("u4");   // [N m]
//     float mg = gravity * drone_mass; // [N]
//     float wh = sqrtf((mg/(4*kf)));   // [N]

//     float rspeed1 = u1 - u3 + u4 - mg + wh; // front left  [rad/s]
//     float rspeed2 = u1 + u2 - u4 - mg + wh; // front right [rad/s]
//     float rspeed3 = u1 + u3 + u4 - mg + wh; // rear right  [rad/s]
//     float rspeed4 = u1 - u2 - u4 - mg + wh; // rear left   [rad/s]
// }



