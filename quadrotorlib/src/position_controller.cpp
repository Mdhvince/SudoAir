#include "quadrotorlib/position_controller.h"
#include <cmath>

PositionController::PositionController(){ }
PositionController::~PositionController(){ }

void PositionController::control_position(std::array<float, 3> &kp_pos, std::array<float, 3> &kd_pos,
                                      std::array<float, 9> &xyz_state, std::array<float, 9> &xyz_state_des,
                                      std::array<float, 4> &inp_plant,
                                      std::array<float, 9> &angle_state_wf_des,
                                      std::array<float, 3> &pqr_state_des,
                                      float drone_mass, float gravity, const float min_motor_thrust, const float max_motor_thrust){


    for(size_t idx{6}; idx < xyz_state.size(); idx++){
        float err {xyz_state_des.at(idx-6) - xyz_state.at(idx-6)};
        float err_dot {xyz_state_des.at(idx-3) - xyz_state.at(idx-3)};
        xyz_state_des.at(idx) = xyz_state.at(idx) + kd_pos.at(idx-6) * err_dot + kp_pos.at(idx-6) * err; //acc_cmd
    }

    

    float u = drone_mass * (gravity + xyz_state_des.at(8));
    u = std::min(u, max_motor_thrust);
    if(u < min_motor_thrust)
        u = min_motor_thrust;
    
    inp_plant.at(0) = u;

    // update acc cmd with the right u limited by actuators
    for(size_t idx{6}; idx < xyz_state.size(); idx++){
        xyz_state_des.at(idx) = std::min(xyz_state_des.at(idx), (max_motor_thrust / drone_mass) - gravity);
        xyz_state_des.at(idx) = std::max(xyz_state_des.at(idx), (min_motor_thrust / drone_mass) - gravity);
    }


    angle_state_wf_des.at(0) = (1/gravity) * (xyz_state_des.at(6) * sin(angle_state_wf_des.at(2)) - xyz_state_des.at(7) * cos(angle_state_wf_des.at(2))); // phi_cmd
    angle_state_wf_des.at(1) = (1/gravity) * (xyz_state_des.at(6) * cos(angle_state_wf_des.at(2)) + xyz_state_des.at(7) * sin(angle_state_wf_des.at(2))); // theta_cmd
    // yaw in not playing to follow a trajectory so the desired yaw is considered as ok


    /* desired angular velocity in the body fixed frame (p_des, q_des, r_des)
       when working near hover p_des and q_des are 0. Since yaw (psi) in not
       playing to follow the trajectory, we set r_des (yaw angular velocity in
       body-fixed frame = yaw_ddot_desired (yaw angular vel in the inertial frame)
    */
    pqr_state_des.at(2) = angle_state_wf_des.at(5);
}
