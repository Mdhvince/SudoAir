#include <iostream>
#include <memory>
#include <array>
#include <unordered_map>
#include <fstream>

#include <opencv2/opencv.hpp>

#include "quadrotorlib/include/quadrotorlib/position_controller.h"
#include "quadrotorlib/include/quadrotorlib/attitude_controller.h"
// #include "quadrotorlib/include/quadrotorlib/quadrotor.h"

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

void sense_sim(float dt, std::unordered_map<std::string, float> &state, float &x_ff, float &y_ff, float &z_ff){

    if(z_ff != 0.0){
        float delta_dot {z_ff * dt};
        state.at("z_dot") = delta_dot;
        float delta_pos {state.at("z_dot") * dt};
        state.at("z") += delta_pos;
    }
    if(y_ff != 0.0){
        float delta_dot {y_ff * dt};
        state.at("y_dot") = delta_dot;
        float delta_pos {state.at("y_dot") * dt};
        state.at("y") += delta_pos;
    }
    if(x_ff != 0.0){
        float delta_dot {x_ff * dt};
        state.at("x_dot") = delta_dot;
        float delta_pos {state.at("x_dot") * dt};
        state.at("x") += delta_pos;
    }
    
}


int main(){
    float g {9.81};
    float m{0.027};
    std::array<float, 3> inertia {2.3951e-5, 2.3951e-5, 2.3951e-5};
    float max_F{0.5687857};
    float min_F{0.16};

    std::unordered_map<std::string, float> quad_specs {
        {"mass", 0.027}, {"Ixx", 2.3951e-5}, {"Iyy", 2.3951e-5}, {"Izz", 2.3951e-5},
        {"max_thrust", 0.5687857}, {"min_thrust", 0.16}, {"Iyy", 2.3951e-5}, {"Izz", 2.3951e-5},
    };
    

    // From trajectory planner
    float x_des {50.0};
    float y_des {800.0};
    float z_des {300.0};
    float x_dot_des {0.5};
    float z_dot_des {0.5};
    float y_dot_des {0.5};
    
    float dt {1.};

    auto pos_ctrl = make_unique<PositionController>();
    auto att_ctrl = make_unique<AttitudeController>();

    float x_ff {0.0};
    float y_ff {0.0};
    float z_ff {0.0};

    std::unordered_map<std::string, float> state {
        {"x", 0.0}, {"y", 0.0}, {"z", 0.0}, {"x_dot", 0.0}, {"y_dot", 0.0}, {"z_dot", 0.0}, 
        {"phi", 0.0}, {"theta", 0.0}, {"psi", 0.0}, {"p", 0.0}, {"q", 0.0}, {"r", 0.0}
    };

    std::unordered_map<std::string, float> state_des {
        {"x", x_des}, {"y", y_des}, {"z", z_des}, {"x_dot", x_dot_des}, {"y_dot", y_dot_des}, {"z_dot", z_dot_des}, 
        {"x_ddot", 0.0}, {"y_ddot", 0.0}, {"z_ddot", 0.0}, {"phi", 0.0}, {"theta", 0.0}, {"psi", 0.0},
        {"phi_dot", 0.0}, {"theta_dot", 0.0}, {"psi_dot", 0.0}, {"p", 0.0}, {"q", 0.0}, {"r", 0.0}
    };

    std::unordered_map<std::string, float> inp_plant {
        {"u1", 0.0}, {"u2", 0.0}, {"u3", 0.0}, {"u4", 0.0}
    };

    cv::Mat img = cv::Mat::zeros(cv::Size(1500, 700), CV_8UC3);
    cv::Point start = cv::Point(state.at("y"), state.at("z"));
    circle(img, start, 15, cv::Scalar(255, 0, 0), 1, cv::LINE_8);
    cv::Point goal = cv::Point(y_des, z_des);
    circle(img, goal, 15, cv::Scalar(0, 0, 255), 1, cv::LINE_8);

    std::array<float, 3> kp_pos {10., 10., 100.};
    std::array<float, 3> kd_pos {15., 15., 60.};
    std::array<float, 3> kp_ang {10., 10., 100.};
    std::array<float, 3> kd_ang {15., 15., 60.};

    const uint n_times {10}; // Inner loop n times faster than outer loop

    for(size_t step{0}; step < 3000; step++){
        show(img, step, state, state_des);

        // here I'm simulating that we've reached the desired acceleration so z_ff = z_ddot desired.
        // In real application z_ff (etc.) will come from the accelerometer.
        x_ff = state_des.at("x_ddot");
        y_ff = state_des.at("y_ddot");
        z_ff = state_des.at("z_ddot");

        
        float F = pos_ctrl->thrust_cmd(kp_pos, kd_pos, state, state_des, m, g, min_F, max_F, z_ff);
        pos_ctrl->angle_cmd(kp_pos, kd_pos, state, state_des, m, g, min_F, max_F, x_ff, y_ff);

        // Attitude controller
        for(size_t i{0}; i < n_times; i++){
            att_ctrl->control_attitude(kp_ang, kd_ang, state, state_des, inp_plant, inertia);
            sense_sim(dt/n_times, state, x_ff, y_ff, z_ff);
        }        
        
    }

    cv::waitKey(0);
    std::cout<< "\n\n";
    return 0;
}