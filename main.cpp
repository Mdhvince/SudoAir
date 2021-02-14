#include <iostream>
#include <memory>
#include <array>
#include <unordered_map>

#include <opencv2/opencv.hpp>

#include "quadrotorlib/include/quadrotorlib/position_controller.h"
#include "quadrotorlib/include/quadrotorlib/attitude_controller.h"
#include <eigen/Eigen/Dense>

using std::unique_ptr;
using std::make_unique;


void show(cv::Mat img, int step, std::unordered_map<std::string, float> &state, std::unordered_map<std::string, float> &state_des){

    if(step % 50 == 0){
        cv::Point p = cv::Point(state.at("y"), state.at("z"));
        circle(img, p, 3, cv::Scalar(0, 255, 0), 1, cv::LINE_8);
    }

    if((state.at("y") > 795 && state.at("y") < 805 && state.at("z") > 295 && state.at("z") < 305)){
        state_des.at("z") = 100;
        state_des.at("y") = 600;
        cv::Point goal = cv::Point(state_des.at("y"), state_des.at("z"));
        circle(img, goal, 15, cv::Scalar(0, 0, 255), 1, cv::LINE_8);
    }
    if(state.at("y") > 595 && state.at("y") < 605 && state.at("z") > 95 && state.at("z") < 105){
        state_des.at("z") = 500;
        state_des.at("y") = 200;
        cv::Point goal = cv::Point(state_des.at("y"), state_des.at("z"));
        circle(img, goal, 15, cv::Scalar(0, 0, 255), 1, cv::LINE_8);
    }
    if(state.at("y") > 195 && state.at("y") < 205 && state.at("z") > 495 && state.at("z") < 505){
        state_des.at("z") = 290;
        state_des.at("y") = 50;
        cv::Point goal = cv::Point(state_des.at("y"), state_des.at("z"));
        circle(img, goal, 15, cv::Scalar(0, 0, 255), 1, cv::LINE_8);
    }

    cv::namedWindow("Image",cv::WINDOW_AUTOSIZE);
    cv::flip(img, img, 0);
    cv::imshow("Image", img);
    cv::waitKey(1);
    cv::flip(img, img, 0);
}

void show2(cv::Mat img, int step, Eigen::Vector3f &pos){

    if(step % 50 == 0){
        cv::Point p = cv::Point(pos(1), pos(2));
        circle(img, p, 3, cv::Scalar(0, 255, 0), 1, cv::LINE_8);
    }

    cv::namedWindow("Image",cv::WINDOW_AUTOSIZE);
    cv::flip(img, img, 0);
    cv::imshow("Image", img);
    cv::waitKey(1);
    cv::flip(img, img, 0);
}


Eigen::Matrix3f rot_mat(Eigen::Vector3f &ang){
    Eigen::Matrix3f R;
    R = Eigen::AngleAxisf(ang(2), Eigen::Vector3f::UnitZ())
        * Eigen::AngleAxisf(ang(1), Eigen::Vector3f::UnitY())
        * Eigen::AngleAxisf(ang(0), Eigen::Vector3f::UnitX());
    return R;
}

Eigen::Vector3f euler_derivatives(Eigen::Vector3f &ang, Eigen::Vector3f &pqr){
    
    Eigen::Matrix3f euler_rot_mat;
    euler_rot_mat << 1, sinf(ang(0)) * tanf(ang(1)), cosf(ang(0)) * tanf(ang(1)),
                     0, cosf(ang(0)), -sinf(ang(0)),
                     0, sinf(ang(0)) / cosf(ang(1)), cosf(ang(0)) / cosf(ang(1));

    Eigen::Vector3f ang_vel = euler_rot_mat * pqr;

    return ang_vel;
}

Eigen::Vector4f set_propeller_angular_velocities(Eigen::Vector3f &acc_cmd, Eigen::Vector3f &pqr_acc,
                                                 Eigen::Vector3f &inertia, float m, float l){
    
    float kf = 1.;
    float km = 1.;
    float F = -acc_cmd(2) * m / kf;

    // std::cout<<F<<std::endl;

    Eigen::Vector3f pqr_no_dim;
    pqr_no_dim << inertia(0) * (pqr_acc(0) / (kf * l)),
                  inertia(1) * (pqr_acc(1) / (kf * l)),
                  inertia(2) * (pqr_acc(2) / km);


    float omega_4_square = (F + pqr_no_dim(0) - pqr_no_dim(1) - pqr_no_dim(2)) / 4.;
    float omega_3_square = (F - pqr_no_dim(1)) / 2 - omega_4_square;
    float omega_2_square = (F - pqr_no_dim(0)) / 2 - omega_3_square;
    float omega_1_square =  F - omega_2_square - omega_3_square - omega_4_square;
    
    Eigen::Vector4f omega_motor_cmd;
    omega_motor_cmd << -sqrtf(omega_1_square),
                       sqrtf(omega_2_square),
                       -sqrtf(omega_3_square),
                       sqrtf(omega_4_square);
    
    return omega_motor_cmd;
}


int main(){
    float g {9.81};
    float m {0.027}; // approx l^3
    float l {92.e-3 / (2*sqrtf(2))};
    float max_F{0.5687857};
    float min_F{0.16};

    std::unordered_map<std::string, float> quad_specs {
        {"mass", 0.027}, {"Ixx", 2.3951e-5}, {"Iyy", 2.3951e-5}, {"Izz", 2.3951e-5},
        {"max_thrust", 0.5687857}, {"min_thrust", 0.16}, {"Iyy", 2.3951e-5}, {"Izz", 2.3951e-5},
    };
    
    float dt {1.};
    auto pos_ctrl = make_unique<PositionController>();
    auto att_ctrl = make_unique<AttitudeController>();

    Eigen::Vector3f kp(10., 10., 100.);
    Eigen::Vector3f kd(15., 15., 60.);
    Eigen::Vector3f pos(0., 0., 0.); // init
    Eigen::Vector3f vel(0., 0., 0.); // init
    Eigen::Vector3f ff_acc(0., 0., 0.); // init
    Eigen::Vector3f pos_des(50., 800., 300.); // traj planner
    Eigen::Vector3f vel_des(0.5, 0.5, 0.5); // traj planner

    Eigen::Vector3f kp_ang(10., 10., 100.);
    Eigen::Vector3f kd_ang(15., 15., 60.);
    Eigen::Vector3f kp_pqr(10., 10., 100.);
    Eigen::Vector3f kd_pqr(15., 15., 60.);
    Eigen::Vector3f ang(0., 0., 0.);
    Eigen::Vector3f ang_des(0., 0., 0.);
    Eigen::Vector3f pqr(0., 0., 0.);
    Eigen::Vector3f pqr_cmd(0., 0., 0.);


    Eigen::Vector3f inertia;
    inertia << 2.3951e-5,
               2.3951e-5,
               2.3951e-5;
    
    float kf = 1.;
    float km = 1.;
    
    cv::Mat img = cv::Mat::zeros(cv::Size(1500, 700), CV_8UC3);
    cv::Point start = cv::Point(pos(1), pos(2));
    circle(img, start, 15, cv::Scalar(255, 0, 0), 1, cv::LINE_8);
    cv::Point goal = cv::Point(pos_des(1), pos_des(2));
    circle(img, goal, 15, cv::Scalar(0, 0, 255), 1, cv::LINE_8);


    const uint n_times {10}; // Inner loop n times faster than outer loop
    float dt_att = dt/n_times;

    for(size_t step{0}; step < 100; step++){

        std::cout<<pos.transpose()<<std::endl;

        Eigen::Matrix3f R = rot_mat(ang);
        Eigen::Vector3f acc_cmd = pos_ctrl->acceleration_cmd(kp, kd, pos, vel, ff_acc, pos_des, vel_des);

        float thrust = pos_ctrl->altitude(acc_cmd, m, g, min_F, max_F, R);
        thrust = std::clamp(thrust, static_cast<float>(min_F*4), static_cast<float>(max_F*4));

        Eigen::Vector3f M = att_ctrl->attitude(kp_ang, kd_ang, kp_pqr, kd_pqr, ang, ang_des, pqr, pqr_cmd, acc_cmd, R, inertia, thrust, m);
        Eigen::Vector4f cmd_4thrusts = att_ctrl->apply_rotor_speed(l, M, thrust);

        // to update
        // pos, ang, vel, pqr
    }

    cv::waitKey(0);
    std::cout<< "\n\n";
    return 0;
}