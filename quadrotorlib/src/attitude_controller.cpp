#include <algorithm>
#include <cmath>
#include "quadrotorlib/attitude_controller.h"

AttitudeController::AttitudeController(){ }
AttitudeController::~AttitudeController(){ }


void AttitudeController::desired_torque(std::string angle, float kp, float kd, float inertia,
                                      std::unordered_map<std::string, float> &state,
                                      std::unordered_map<std::string, float> &state_des,
                                      std::unordered_map<std::string, float> &inp_plant){
    
    std::string vel {};
    std::string moment {};
    if(angle == "phi") {vel = "p"; moment = "u2";}
    else if (angle == "theta") {vel = "q"; moment = "u3";}
    else if (angle == "psi") {vel = "r"; moment = "u4";}
    else {std::cerr<< "Expected angle phi, theta or psi. Got " << angle << " instead."<<std::endl;}

    float err {state_des.at(angle) - state.at(angle)};
    float err_dot {state_des.at(vel) - state.at(vel)};
    inp_plant.at(moment) = inertia * (kp * err + kd * err_dot);
}


void AttitudeController::control_attitude(std::array<float, 3> &kp_ang, std::array<float, 3> &kd_ang,
                                          std::unordered_map<std::string, float> &state, std::unordered_map<std::string, float> &state_des,
                                          std::unordered_map<std::string, float> &inp_plant, std::array<float, 3> &inertia){

    desired_torque("phi", kp_ang.at(0), kd_ang.at(0), inertia.at(0), state, state_des, inp_plant);
    desired_torque("theta", kp_ang.at(1), kd_ang.at(1), inertia.at(1), state, state_des, inp_plant);
    desired_torque("psi", kp_ang.at(2), kd_ang.at(2), inertia.at(2), state, state_des, inp_plant);

}

// thanks to equation (6) and (7) from : http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.169.1687&rep=rep1&type=pdf
// the equation is established at near the hover state. That mean p = phi_dot, q = theta_dot & r = psi_dot
void AttitudeController::apply_rotor_speed(std::unordered_map<std::string, float> &inp_plant, float kf, float drone_mass, float gravity){

    float u1 = inp_plant.at("u1");
    float u2 = inp_plant.at("u2");
    float u3 = inp_plant.at("u3");
    float u4 = inp_plant.at("u4");
    float mg = gravity * drone_mass;
    float wh = sqrtf((mg/(4*kf)));

    float rspeed1 = u1 - u3 + u4 - mg + wh;
    float rspeed2 = u1 + u2 - u4 - mg + wh;
    float rspeed3 = u1 + u3 + u4 - mg + wh;
    float rspeed4 = u1 - u2 - u4 - mg + wh;

    // std::array<float, 4> desired_rspeed {rspeed1, rspeed2, rspeed3, rspeed4};
}



