#include <iostream>
#include <memory>
#include <array>
#include <unordered_map>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

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

void integrate_twice_from(std::string acc, float dt, std::unordered_map<std::string, float> &state, std::unordered_map<std::string, float> &state_des){
    std::string vel {};
    std::string pos {};
    if(acc == "x_ddot") {vel = "x_dot"; pos = "x";}
    else if (acc == "y_ddot") {vel = "y_dot"; pos = "y";}
    else if (acc == "z_ddot") {vel = "z_dot"; pos = "z";}
    else if (acc == "p") {vel = "phi_dot"; pos = "phi";}
    else if (acc == "q") {vel = "theta_dot"; pos = "theta";}
    else if (acc == "r") {vel = "psi_dot"; pos = "psi";}
    else {std::cerr<< "Expected acc x_ddot, y_ddot or z_ddot. Got " << acc << " instead."<<std::endl;}

    if(state_des.at(acc) != 0.0){
        float delta_dot {state_des.at(acc) * dt};
        state.at(vel) = delta_dot;
        float delta_pos {state.at(vel) * dt};
        state.at(pos) += delta_pos;
    }
}

void simulate_displacement(float dt, std::unordered_map<std::string, float> &state, std::unordered_map<std::string, float> &state_des){

    std::array<std::string, 6> acc_names {"x_ddot", "y_ddot", "z_ddot", "p", "q", "r"};
    for(auto &acc_name: acc_names)
        integrate_twice_from(acc_name, dt, state, state_des);
}


int main(){
    float gravity {9.81};
    float drone_mass_KG{0.027};
    std::array<float, 3> inertia {2.3951e-5, 2.3951e-5, 2.3951e-5};
    float max_motor_thrust_N{0.5687857};
    float min_motor_thrust_N{(drone_mass_KG * gravity) - static_cast<float>(.1)};

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

    std::unordered_map<std::string, float> state {
        {"x", 0.0}, {"y", 0.0}, {"z", 0.0}, 
        {"x_dot", 0.0}, {"y_dot", 0.0}, {"z_dot", 0.0}, 
        {"x_ddot", 0.0}, {"y_ddot", 0.0}, {"z_ddot", 0.0}, 
        {"phi", 0.0}, {"theta", 0.0}, {"psi", 0.0},
        {"phi_dot", 0.0}, {"theta_dot", 0.0}, {"psi_dot", 0.0},
        {"p", 0.0}, {"q", 0.0}, {"r", 0.0}
    };

    std::unordered_map<std::string, float> state_des {
        {"x", x_des}, {"y", y_des}, {"z", z_des}, 
        {"x_dot", x_dot_des}, {"y_dot", y_dot_des}, {"z_dot", z_dot_des}, 
        {"x_ddot", 0.0}, {"y_ddot", 0.0}, {"z_ddot", 0.0}, 
        {"phi", 0.0}, {"theta", 0.0}, {"psi", 0.0},
        {"phi_dot", 0.0}, {"theta_dot", 0.0}, {"psi_dot", 0.0},
        {"p", 0.0}, {"q", 0.0}, {"r", 0.0}
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

    /*
    auto sudoAir = make_unique<QuadRotor>(m, inertia, max_motor_thrust_N, min_motor_thrust_N, xyz_state);
    sudoAir->start_rotors();

    while(true){
        next_wp = get_next_waypoint();

        if(next_wp == home_wp && waypoint_reached(home_wp)){
            sudoAir->stop_rotors();
            break;
        }

        while(!waypoint_reached(next_wp)){
            sudoAir->control_position();
            sudoAir->send_cmd()
        }
    }

    */

    for(size_t step{0}; step < 3000; step++){
                
        pos_ctrl->control_altitude(kp_pos, kd_pos, state, state_des, inp_plant,
                                   drone_mass_KG, gravity, min_motor_thrust_N, max_motor_thrust_N);

        pos_ctrl->control_lateral(kp_pos, kd_pos, state, state_des,
                                  drone_mass_KG, gravity, min_motor_thrust_N, max_motor_thrust_N);

        // Attitude controller
        for(size_t i{0}; i < n_times; i++){
            att_ctrl->control_attitude(kp_ang, kd_ang, state, state_des, inp_plant, inertia);
            att_ctrl->apply_rotor_speed(inp_plant, 1.0, drone_mass_KG, gravity);
            simulate_displacement(dt/n_times, state, state_des);
        }
        show(img, step, state, state_des);
    }

    cv::waitKey(0);
    std::cout<< "\n\n";
    return 0;
}