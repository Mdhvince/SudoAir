#ifndef _POSITION_CONTROLLER_H_
#define _POSITION_CONTROLLER_H_
#include <iostream>
#include <array>
#include <unordered_map>
#include <eigen/Eigen/Dense>


class PositionController{

public:
    PositionController();
    ~PositionController();

    Eigen::Vector3f acceleration_cmd(Eigen::Vector3f &kp, Eigen::Vector3f &kd, Eigen::Vector3f &pos, Eigen::Vector3f &vel,
                                     Eigen::Vector3f &ff_acc, Eigen::Vector3f &pos_des, Eigen::Vector3f &vel_des);
    
    float altitude(Eigen::Vector3f &acc_cmd, float m, float g, const float min_F, const float max_F, Eigen::Matrix3f &R);
    void lateral(Eigen::Vector3f &acc_cmd, float m, float g, const float min_F, const float max_F);

    
    float desired_acc(std::string axis, float kp, float kd,
                       std::unordered_map<std::string, float> &state,
                       std::unordered_map<std::string, float> &state_des, float &ff_term);


    float thrust_cmd(std::array<float, 3> &kp_pos, std::array<float, 3> &kd_pos,
                           std::unordered_map<std::string, float> &state, std::unordered_map<std::string, float> &state_des,
                           float m, float g, const float min_F, const float max_F, float &z_ff);

    void angle_cmd(std::array<float, 3> &kp_pos, std::array<float, 3> &kd_pos,
                          std::unordered_map<std::string, float> &state, std::unordered_map<std::string, float> &state_des,
                          float m, float g, const float min_F, const float max_F, float &x_ff, float &y_ff);
    
};
#endif