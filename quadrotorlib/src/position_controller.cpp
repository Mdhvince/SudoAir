#include "quadrotorlib/position_controller.h"
#include <algorithm>
#include <cmath>

PositionController::PositionController(){ }
PositionController::~PositionController(){ }


Eigen::Vector3f PositionController::acceleration_cmd(Eigen::Vector3f &kp, Eigen::Vector3f &kd, Eigen::Vector3f &pos, Eigen::Vector3f &vel,
                                                     Eigen::Vector3f &ff_acc, Eigen::Vector3f &pos_des, Eigen::Vector3f &vel_des){
    
    // (receive x, y, z, x_vel, y_vel, z_vel from the drone)
    Eigen::Vector3f acc_cmd = ff_acc + kd.cwiseProduct((vel_des - vel)) + kp.cwiseProduct((pos_des - pos));
    return acc_cmd;
}


float PositionController::altitude(Eigen::Vector3f &acc_cmd, float m, float g, const float min_F, const float max_F, Eigen::Matrix3f &R){
    
    float max_acc = ((max_F*4.) / m) - g;
    float min_acc = ((min_F*4.) / m) - g;

    float acc = (acc_cmd(2) - g) / R(2, 2);

    float thrust = -m * std::clamp(acc, min_acc, max_acc); // Z is down
    
    return thrust;
}