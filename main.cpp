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

void draw_pid_response(cv::Mat img, int step, float z, float y){
    cv::Point p = cv::Point(y, z); // (cols, rows)
    circle(img, p, 3, cv::Scalar(0, 255, 0), 1, cv::LINE_8);
}

// Z X Y
cv::Mat R(float phi, float theta, float psi){
    
    cv::Mat rx = (cv::Mat_<float>(3,3) << 
                   1, 0, 0,
                   0, cos(phi), -sin(phi),
                   0, sin(phi),  cos(phi));

    cv::Mat ry = (cv::Mat_<float>(3,3) << 
                   cos(theta), 0, sin(theta),
                   0, 1, 0,
                  -sin(theta), 0, cos(theta));
    
    cv::Mat rz = (cv::Mat_<float>(3,3) << 
                   cos(psi), -sin(psi), 0,
                   sin(psi), cos(psi), 0,
                   0, 0, 1);
    
    cv::Mat r_yx = ry * rx;
     
    return rz * r_yx;
}

void simulate_displacement(float dt,
                           std::array<float, 9> &xyz_state, std::array<float, 9> &xyz_state_des,
                           std::array<float, 9> &angle_state_wf, std::array<float, 9> &angle_state_wf_des){

    for(size_t idx{0}; idx <= 2; idx++){
        if(xyz_state_des.at(idx+6) != 0.0){
            float delta_dot {xyz_state_des.at(idx+6) * dt};
            xyz_state.at(idx+3) = delta_dot;
            float delta_pos {xyz_state.at(idx+3) * dt};
            xyz_state.at(idx) += delta_pos;
        }
    }

    for(size_t idx{0}; idx <= 2; idx++){
        if(angle_state_wf_des.at(idx+6) != 0.0){
            float delta_angle_dot {angle_state_wf_des.at(idx+6) * dt};
            angle_state_wf.at(idx+3) = delta_angle_dot;
            float delta_angle {angle_state_wf.at(idx+3) * dt};
            angle_state_wf.at(idx) += delta_angle;
        }
    }

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
        {"p", 0.0}, {"q", 0.0}, {"r", 0.0}
    };

    std::unordered_map<std::string, float> state_des {
        {"x", x_des}, {"y", y_des}, {"z", z_des}, 
        {"x_dot", x_dot_des}, {"y_dot", y_dot_des}, {"z_dot", z_dot_des}, 
        {"x_ddot", 0.0}, {"y_ddot", 0.0}, {"z_ddot", 0.0}, 
        {"phi", 0.0}, {"theta", 0.0}, {"psi", 0.0},
        {"p", 0.0}, {"q", 0.0}, {"r", 0.0}
    };

    // std::unordered_map<std::string, float> inp_plant {
    //     {"u1", 0.0}, {"u2", 0.0}, {"u3", 0.0}, {"u4", 0.0}
    // };

    

    /*
    state
    des_state
    */
    std::array<float, 9> xyz_state {0.0, 100.0, 80.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<float, 3> pqr_state {0.0, 0.0, 0.0};
    std::array<float, 9> angle_state_wf {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    std::array<float, 9> xyz_state_des {x_des, y_des, z_des, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<float, 9> angle_state_wf_des {0.0, 0.0, 3.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<float, 3> pqr_state_des {0.0, 0.0, 0.0};

    std::array<float, 4> inp_plant {0.0, 0.0, 0.0, 0.0};


    cv::Mat img = cv::Mat::zeros(cv::Size(1500, 700), CV_8UC3);
    cv::Point start = cv::Point(xyz_state.at(1), xyz_state.at(2));
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
                
        pos_ctrl->control_altitude(kp_pos, kd_pos,
                                   xyz_state, xyz_state_des,
                                   inp_plant, angle_state_wf_des, pqr_state_des,
                                   drone_mass_KG, gravity, min_motor_thrust_N, max_motor_thrust_N);

        pos_ctrl->control_lateral(xyz_state_des, angle_state_wf_des, pqr_state_des, gravity);

        // Attitude controller
        for(size_t i{0}; i < n_times; i++){
            att_ctrl->control_attitude(kp_ang, kd_ang, pqr_state, pqr_state_des, angle_state_wf, angle_state_wf_des, inp_plant, inertia);
            att_ctrl->apply_rotor_speed(inp_plant, 1.0, drone_mass_KG, gravity);

            simulate_displacement(dt/n_times, xyz_state, xyz_state_des, angle_state_wf, angle_state_wf_des);
        }







        if(step % 50 == 0)
            draw_pid_response(img, step, xyz_state.at(2), xyz_state.at(1));
        

        if((xyz_state.at(1) > 795 && xyz_state.at(1) < 805 && xyz_state.at(2) > 295 && xyz_state.at(2) < 305)){
            xyz_state_des.at(2) = 100;
            xyz_state_des.at(1) = 600;
            cv::Point goal = cv::Point(xyz_state_des.at(1), xyz_state_des.at(2));
            circle(img, goal, 15, cv::Scalar(0, 0, 255), 1, cv::LINE_8);
        }
        if(xyz_state.at(1) > 595 && xyz_state.at(1) < 605 && xyz_state.at(2) > 95 && xyz_state.at(2) < 105){
            xyz_state_des.at(2) = 500;
            xyz_state_des.at(1) = 200;
            cv::Point goal = cv::Point(xyz_state_des.at(1), xyz_state_des.at(2));
            circle(img, goal, 15, cv::Scalar(0, 0, 255), 1, cv::LINE_8);
        }
        if(xyz_state.at(1) > 195 && xyz_state.at(1) < 205 && xyz_state.at(2) > 495 && xyz_state.at(2) < 505){
            xyz_state_des.at(2) = 290;
            xyz_state_des.at(1) = 50;
            cv::Point goal = cv::Point(xyz_state_des.at(1), xyz_state_des.at(2));
            circle(img, goal, 15, cv::Scalar(0, 0, 255), 1, cv::LINE_8);
        }
    }
    

    cv::namedWindow("Image",cv::WINDOW_AUTOSIZE);
    cv::flip(img, img, 0);
    cv::imshow("Image", img);
    cv::waitKey(0);

    std::cout<< "\n\n";
    return 0;
}