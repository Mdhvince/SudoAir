#include "quadrotorlib/position_controller.h"
#include <algorithm>
#include <cmath>

PositionController::PositionController(){ }
PositionController::~PositionController(){ }

void PositionController::control_altitude(std::array<float, 3> &kp_pos, std::array<float, 3> &kd_pos,
                                      std::array<float, 9> &xyz_state, std::array<float, 9> &xyz_state_des,
                                      std::array<float, 4> &inp_plant,
                                      std::array<float, 9> &angle_state_wf_des,
                                      std::array<float, 3> &pqr_state_des,
                                      float drone_mass, float gravity, const float min_motor_thrust, const float max_motor_thrust){
                                        
    float max_acc = ((max_motor_thrust - static_cast<float>(.01)) / drone_mass) - gravity;
    float min_acc = (min_motor_thrust / drone_mass) - gravity;

    for(size_t idx{6}; idx < xyz_state.size(); idx++){
        float err {xyz_state_des.at(idx-6) - xyz_state.at(idx-6)};

        // we can vary our desired velocity in fonction of position error OR keep a constant desired velocity
        float velocity_des = xyz_state_des.at(idx-3) + kp_pos.at(idx-6) * err;
        xyz_state_des.at(idx-3) = std::clamp(velocity_des, (float)0.0, (float)0.3);

        float err_dot {xyz_state_des.at(idx-3) - xyz_state.at(idx-3)};

        // acc_cmd saturated (x, y, z)
        float acc_des = xyz_state.at(idx) + kd_pos.at(idx-6) * err_dot + kp_pos.at(idx-6) * err;
        xyz_state_des.at(idx) = std::clamp(acc_des, min_acc, max_acc);
    }

    float z_ddot {xyz_state_des.at(8)};
    inp_plant.at(0) = drone_mass * (gravity + z_ddot); // Sum of the thrust U
        
    

    float psi {angle_state_wf_des.at(2)};
    float x_ddot {xyz_state_des.at(6)};
    float y_ddot {xyz_state_des.at(7)};

    angle_state_wf_des.at(0) = (1/gravity) * (x_ddot * sin(psi) - y_ddot * cos(psi)); // phi_cmd
    angle_state_wf_des.at(1) = (1/gravity) * (x_ddot * cos(psi) + y_ddot * sin(psi)); // theta_cmd
    // yaw in not playing to follow a trajectory so the desired yaw is considered as ok


    /* desired angular velocity in the body fixed frame (p_des, q_des, r_des)
       when working near hover p_des and q_des are 0. Since yaw (psi) in not
       playing to follow the trajectory, we set r_des (yaw angular velocity in
       body-fixed frame = yaw_ddot_desired (yaw angular vel in the inertial frame)
    */
    pqr_state_des.at(2) = angle_state_wf_des.at(5);
}

void PositionController::control_lateral(std::array<float, 9> &xyz_state_des, std::array<float, 9> &angle_state_wf_des,
                    std::array<float, 3> &pqr_state_des, float gravity){
    
    float psi {angle_state_wf_des.at(2)};
    float x_ddot {xyz_state_des.at(6)};
    float y_ddot {xyz_state_des.at(7)};

    angle_state_wf_des.at(0) = (1/gravity) * (x_ddot * sin(psi) - y_ddot * cos(psi)); // phi_cmd
    angle_state_wf_des.at(1) = (1/gravity) * (x_ddot * cos(psi) + y_ddot * sin(psi)); // theta_cmd
    // yaw in not playing to follow a trajectory so the desired yaw is considered as ok


    /*desired angular velocity in the body fixed frame (p_des, q_des, r_des)
    when working near hover p_des and q_des are 0. Since yaw (psi) in not
    playing to follow the trajectory, we set r_des (yaw angular velocity in
    body-fixed frame = yaw_ddot_desired (yaw angular vel in the inertial frame)*/
    pqr_state_des.at(2) = angle_state_wf_des.at(5); 

}
