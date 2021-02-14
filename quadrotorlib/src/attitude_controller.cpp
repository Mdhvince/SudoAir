#include <algorithm>
#include <cmath>
#include "quadrotorlib/attitude_controller.h"

AttitudeController::AttitudeController(){ }
AttitudeController::~AttitudeController(){ }


Eigen::Vector3f AttitudeController::attitude(Eigen::Vector3f &kp_ang, Eigen::Vector3f &kd_ang,
                                             Eigen::Vector3f &kp_pqr, Eigen::Vector3f &kd_pqr,
                                             Eigen::Vector3f &ang, Eigen::Vector3f &ang_des,
                                             Eigen::Vector3f &pqr, Eigen::Vector3f &pqr_cmd,
                                             Eigen::Vector3f &acc_cmd, Eigen::Matrix3f &R,
                                             Eigen::Vector3f &inertia, float thrust, float m){
    
    // roll pitch (receive phi theta from the drone)
    if (thrust > 0) {
        thrust = -thrust / m;
        
        float min_angle = -45. * (M_PIf32/180.);
        float max_angle = 45. * (M_PIf32/180.);
        float phi_cmd_dot = kp_ang(0) * (std::clamp(acc_cmd(0) / thrust, min_angle, max_angle) - R(0, 2));
        float theta_cmd_dot = kp_ang(1) * (std::clamp(acc_cmd(1) / thrust, min_angle, max_angle) - R(1, 2));

        Eigen::Matrix<float, 2, 2> rot_w2b;
        rot_w2b << R(1, 0), -R(0, 0), 
                R(1, 1), -R(0, 1);
        
        rot_w2b /= R(2, 2);

        Eigen::Vector2f ang_vel;
        ang_vel << phi_cmd_dot, theta_cmd_dot;

        Eigen::Vector2f pq_cmd;
        pq_cmd = rot_w2b * ang_vel;

        pqr_cmd(0) = pq_cmd(0);
        pqr_cmd(1) = pq_cmd(1);
    }
    else{
        pqr_cmd(0) = 0.0;
        pqr_cmd(1) = 0.0;
    }
    
    // yaw (receive psi from the drone)
    ang_des(2) = std::clamp(ang_des(2), -2*M_PIf32, 2*M_PIf32);

    float err_yaw = ang_des(2) - ang(2);
    if (err_yaw > M_PIf32)
        err_yaw -= 2 * M_PIf32;

    if (err_yaw < -M_PIf32)
        err_yaw += 2 * M_PIf32;
    
    pqr_cmd(2) = kp_ang(2) * err_yaw;


    // body rate (receive pqr from the drone)
    Eigen::Vector3f M = inertia.cwiseProduct(kp_pqr.cwiseProduct(pqr_cmd - pqr));

    return M;
}


Eigen::Vector4f AttitudeController::apply_rotor_speed(float l, Eigen::Vector3f &M, float thrust){

    float drag_thrust_ratio = 1.;                                    
    float t1 = M(0) / l;
    float t2 = M(1) / l;
    float t3 = -M(2) / drag_thrust_ratio;
    float t4 = thrust;

    Eigen::Vector4f cmd_N;
    cmd_N << (t1 + t2 + t3 + t4)/4.,  // front left  - f1
             (-t1 + t2 - t3 + t4)/4., // front right - f2
             (t1 - t2 - t3 + t4)/4.,  // rear left   - f4
             (-t1 - t2 + t3 + t4)/4.f; // rear right  - f3

    return cmd_N;
}



