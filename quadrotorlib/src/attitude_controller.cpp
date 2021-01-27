#include <algorithm>
#include "quadrotorlib/attitude_controller.h"

AttitudeController::AttitudeController(){ }
AttitudeController::~AttitudeController(){ }


void AttitudeController::control_attitude(std::array<float, 3> &kp_ang, std::array<float, 3> &kd_ang,
                                       std::array<float, 3> &pqr_state, std::array<float, 3> &pqr_state_des,
                                       std::array<float, 9> &angle_state_wf, std::array<float, 9> &angle_state_wf_des,
                                       std::array<float, 4> &inp_plant, std::array<float, 3> &inertia){
    // attitude contrl
    for(u_int idx{0}; idx < pqr_state.size(); idx++){
        float err {angle_state_wf_des.at(idx) - angle_state_wf.at(idx)};
        float err_dot {pqr_state_des.at(idx) - pqr_state.at(idx)};
        inp_plant.at(idx+1) = inertia.at(idx) * (kp_ang.at(idx) * err + kd_ang.at(idx) * err_dot); // moment u2 u3 u4
    }
}



