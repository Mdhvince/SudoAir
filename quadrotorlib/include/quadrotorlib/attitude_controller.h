#ifndef _ATTITUDE_CONTROLLER_H_
#define _ATTITUDE_CONTROLLER_H_
#include <iostream>
#include <eigen/Eigen/Dense>

class AttitudeController{

public:
    AttitudeController();
    AttitudeController(const AttitudeController &source);
    ~AttitudeController();

    Eigen::Vector3f attitude(Eigen::Vector3f &kp_ang, Eigen::Vector3f &kd_ang,
                             Eigen::Vector3f &kp_pqr, Eigen::Vector3f &kd_pqr,
                             Eigen::Vector3f &ang, Eigen::Vector3f &ang_des,
                             Eigen::Vector3f &pqr, Eigen::Vector3f &pqr_cmd,
                             Eigen::Vector3f &acc_cmd, Eigen::Matrix3f &R, Eigen::Vector3f &inertia, float thrust, float m);
    
    Eigen::Vector4f apply_rotor_speed(float l, Eigen::Vector3f &M, float thrust);
    
};
#endif
