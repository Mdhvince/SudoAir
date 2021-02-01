#include <algorithm>
#include <cmath>
#include "quadrotorlib/attitude_controller.h"

AttitudeController::AttitudeController(){ }
AttitudeController::~AttitudeController(){ }


void AttitudeController::control_body_rate(std::array<float, 3> &inertia,
                                      std::unordered_map<std::string, float> &state,
                                      std::unordered_map<std::string, float> &state_des,
                                      std::unordered_map<std::string, float> &inp_plant){
    
    float kp {100};
    float error_p {state_des.at("p") - state.at("p")};
    inp_plant.at("u2") = inertia.at(0) * (kp * error_p);

    float error_q {state_des.at("q") - state.at("q")};
    inp_plant.at("u3") = inertia.at(1) * (kp * error_q);

    float error_r {state_des.at("r") - state.at("r")};
    inp_plant.at("u4") = inertia.at(2) * (kp * error_r);

}

void AttitudeController::control_attitude(std::array<float, 3> &kp_ang, std::array<float, 3> &kd_ang,
                                          std::unordered_map<std::string, float> &state, std::unordered_map<std::string, float> &state_des,
                                          std::unordered_map<std::string, float> &inp_plant, std::array<float, 3> &inertia){
    
    control_yaw(state, state_des);
    control_roll_pitch(state, state_des);
    control_body_rate(inertia, state, state_des, inp_plant);
}

void AttitudeController::control_yaw(std::unordered_map<std::string, float> &state, std::unordered_map<std::string, float> &state_des){

    // psi_cmd = psi_des
    float kp = 2.0;
    state_des.at("r") = kp * (state_des.at("psi") - state.at("psi")); // r_cmd
}

void AttitudeController::control_roll_pitch(std::unordered_map<std::string, float> &state, std::unordered_map<std::string, float> &state_des){

    float phi_cmd = state_des.at("phi");
    float theta_cmd = state_des.at("theta");
    float psi_cmd = state_des.at("psi");

    float phi = state.at("phi");
    float theta = state.at("theta");
    float psi = state.at("psi");

    std::array <std::array<float, 3>, 3> R = Rot_mat(phi, theta, psi);
    std::array <std::array<float, 3>, 3> R_cmd = Rot_mat(phi_cmd, theta_cmd, psi_cmd);

    float bx_cmd = R_cmd[0][2];
    float by_cmd = R_cmd[1][2];

    float bx_err = bx_cmd - R[0][2];
    float by_err = by_cmd - R[1][2];
    float kp_rp = 2;

    float pterm_x = kp_rp * bx_err;
    float pterm_y = kp_rp * by_err;

    state_des.at("p") = (R[1][0] * pterm_x - R[0][0] * pterm_y) / R[2][2]; // p_cmd
    state_des.at("q") = (R[1][1] * pterm_x - R[0][1] * pterm_y) / R[2][2]; // q_cmd
}

std::array <std::array<float, 3>, 3> AttitudeController::Rot_mat(float phi, float theta, float psi){
    
    std::array <std::array<float, 3>, 3> R {{
        {{ cosf(psi)*cosf(theta)-sinf(phi)*sinf(psi)*sinf(theta), -cosf(phi)*sinf(psi), cosf(psi)*sinf(theta)+cosf(theta)*sinf(phi)*sinf(psi) }},
        {{ cosf(theta)*sinf(psi)+cosf(psi)*sinf(phi)*sinf(theta), cosf(phi)*cosf(psi), sinf(psi)*sinf(theta)-cosf(theta)*sinf(phi)*cosf(psi) }},
        {{ -cosf(phi)*sinf(theta), sinf(phi), cosf(phi)*cosf(theta) }}
    }};
    return R;
}



// thanks to equation (6) and (7) from : http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.169.1687&rep=rep1&type=pdf
// the equation is established at near the hover state. That mean p = phi_dot, q = theta_dot & r = psi_dot
void AttitudeController::apply_rotor_speed(std::unordered_map<std::string, float> &inp_plant, float kf, float drone_mass, float gravity){

    float u1 = inp_plant.at("u1");   // [N]
    float u2 = inp_plant.at("u2");   // [N m]
    float u3 = inp_plant.at("u3");   // [N m]
    float u4 = inp_plant.at("u4");   // [N m]
    float mg = gravity * drone_mass; // [N]
    float wh = sqrtf((mg/(4*kf)));   // [N]

    float rspeed1 = u1 - u3 + u4 - mg + wh; // front left  [rad/s]
    float rspeed2 = u1 + u2 - u4 - mg + wh; // front right [rad/s]
    float rspeed3 = u1 + u3 + u4 - mg + wh; // rear right  [rad/s]
    float rspeed4 = u1 - u2 - u4 - mg + wh; // rear left   [rad/s]
}



