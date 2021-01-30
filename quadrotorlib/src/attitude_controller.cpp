#include <algorithm>
#include <cmath>
#include "quadrotorlib/attitude_controller.h"

AttitudeController::AttitudeController(){ }
AttitudeController::~AttitudeController(){ }


void AttitudeController::control_attitude(std::array<float, 3> &kp_ang, std::array<float, 3> &kd_ang,
                                       std::array<float, 3> &pqr_state, std::array<float, 3> &pqr_state_des,
                                       std::array<float, 9> &angle_state_wf, std::array<float, 9> &angle_state_wf_des,
                                       std::array<float, 4> &inp_plant, std::array<float, 3> &inertia){
    // attitude contrl
    for(size_t idx{0}; idx < pqr_state.size(); idx++){
        float err {angle_state_wf_des.at(idx) - angle_state_wf.at(idx)};
        float err_dot {pqr_state_des.at(idx) - pqr_state.at(idx)};
        inp_plant.at(idx+1) = inertia.at(idx) * (kp_ang.at(idx) * err + kd_ang.at(idx) * err_dot); // moment u2 u3 u4
    }
}

// thanks to equation (6) and (7) from : http://citeseerx.ist.psu.edu/viewdoc/download?doi=10.1.1.169.1687&rep=rep1&type=pdf
// the equation is established at near the hover state. That mean p = phi_dot, q = theta_dot & r = psi_dot
void AttitudeController::apply_rotor_speed(std::array<float, 4> &inp_plant, float kf, float drone_mass, float gravity){

    float u1 = inp_plant.at(0);
    float u2 = inp_plant.at(1);
    float u3 = inp_plant.at(2);
    float u4 = inp_plant.at(3);
    float mg = gravity * drone_mass;
    float wh = sqrt((mg/(4*kf)));

    float rspeed1 = u1 - u3 + u4 - mg + wh;
    float rspeed2 = u1 + u2 - u4 - mg + wh;
    float rspeed3 = u1 + u3 + u4 - mg + wh;
    float rspeed4 = u1 - u2 - u4 - mg + wh;

    // std::array<float, 4> desired_rspeed {rspeed1, rspeed2, rspeed3, rspeed4};
}



