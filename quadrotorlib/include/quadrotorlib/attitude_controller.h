#ifndef _ATTITUDE_CONTROLLER_H_
#define _ATTITUDE_CONTROLLER_H_
#include <iostream>
#include <array>

class AttitudeController{

public:
    AttitudeController();
    AttitudeController(const AttitudeController &source);
    ~AttitudeController();


    void control_attitude(std::array<float, 3> &kp_ang, std::array<float, 3> &kd_ang,
                          std::unordered_map<std::string, float> &state, std::unordered_map<std::string, float> &state_des,
                          std::unordered_map<std::string, float> &inp_plant, std::array<float, 3> &inertia);
    
    void apply_rotor_speed(std::unordered_map<std::string, float> &inp_plant, float kf, float drone_mass, float gravity);

    // void desired_torque(std::string angle, float kp, float kd, float inertia,
    //                     std::unordered_map<std::string, float> &state,
    //                     std::unordered_map<std::string, float> &state_des,
    //                     std::unordered_map<std::string, float> &inp_plant);
    
    // void control_body_rate(std::array<float, 3> &inertia,
    //                     std::unordered_map<std::string, float> &state,
    //                     std::unordered_map<std::string, float> &state_des,
    //                     std::unordered_map<std::string, float> &inp_plant);
    
    // void control_yaw(std::unordered_map<std::string, float> &state, std::unordered_map<std::string, float> &state_des);

    // void control_roll_pitch(std::unordered_map<std::string, float> &state, std::unordered_map<std::string, float> &state_des);

    // std::array <std::array<float, 3>, 3> Rot_mat(float phi, float theta, float psi);
};
#endif
