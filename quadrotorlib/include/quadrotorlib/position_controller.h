#ifndef _POSITION_CONTROLLER_H_
#define _POSITION_CONTROLLER_H_
#include <iostream>
#include <array>
#include <unordered_map>


class PositionController{

public:
    PositionController();
    ~PositionController();
    
    std::array <std::array<float, 3>, 3> Rot_mat(float phi, float theta, float psi);


    // non linear controller
    float desired_acc(std::string axis, float kp, float kd,
                       std::unordered_map<std::string, float> &state,
                       std::unordered_map<std::string, float> &state_des, float &ff_term);

    std::array<float, 3> R_b3(float phi, float theta, float psi);

    float thrust_cmd(std::array<float, 3> &kp_pos, std::array<float, 3> &kd_pos,
                           std::unordered_map<std::string, float> &state, std::unordered_map<std::string, float> &state_des,
                           float m, float g, const float min_F, const float max_F, float &z_ff);

    void angle_cmd(std::array<float, 3> &kp_pos, std::array<float, 3> &kd_pos,
                          std::unordered_map<std::string, float> &state, std::unordered_map<std::string, float> &state_des,
                          float m, float g, const float min_F, const float max_F, float &x_ff, float &y_ff);
    
};
#endif