#include <iostream>
#include <memory>
#include <array>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "controllib/include/controllib/pid_controller.h"

using std::unique_ptr;
using std::make_unique;

void draw_pid_response(cv::Mat img, int step, float z, float y){
    cv::Point p = cv::Point(y, z); // (cols, rows)
    circle(img, p, 3, cv::Scalar(0, 255, 0), 1, cv::LINE_8);
}

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

void simulate_displacement(float dt, std::array<float, 5> &state_z, std::array<float, 5> &state_y, std::array<float, 5> &state_phi){
    
    float delta_z_dot {state_z.at(2) * dt};
    state_z.at(1) = delta_z_dot;
    float delta_pos_z {state_z.at(1) * dt};
    state_z.at(0) += delta_pos_z;

    float delta_y_dot {state_y.at(2) * dt};
    state_y.at(1) = delta_y_dot;
    float delta_pos_y {state_y.at(1) * dt};
    state_y.at(0) += delta_pos_y;


    float delta_phi_dot {state_phi.at(2) * dt};
    state_phi.at(1) = delta_phi_dot;
    float delta_phi {state_phi.at(1) * dt};
    state_phi.at(0) += delta_phi;
}


int main(){
    float gravity {9.81};
    float drone_mass_KG{0.2};
    float min_motor_thrust_N{drone_mass_KG * (float)(gravity / 2.0)};
    float max_motor_thrust_N{2 * drone_mass_KG * gravity};

    unique_ptr<PID_Controller> pid = make_unique<PID_Controller>(min_motor_thrust_N, max_motor_thrust_N, drone_mass_KG);

    // trajectory planner
    float z_des {300.0};
    float z_dot_des {0.5};
    float y_des {800.0};
    float y_dot_des {0.5};
    float phi_cmd_dot {0.5};
    float dt {1};

    std::array<float, 5> state_z {0.0, 0.0, 0.0, z_des, z_dot_des};
    std::array<float, 5> state_y {0.0, 0.0, 0.0, y_des, y_dot_des};
    std::array<float, 5> state_phi {0.0, 0.0, 0.0, 0.0, phi_cmd_dot};

    cv::Mat img = cv::Mat::zeros(cv::Size(1500, 700), CV_8UC3);

    for(int step{0}; step < 700; step++){

        float u1 = pid->thrust(state_z, state_phi.at(0));
        float phi_c = pid->phi_cmd(state_y, u1);
        
        state_z.at(2) = (u1 / drone_mass_KG) * cos(state_phi.at(0)); // z_ddot
        state_y.at(2) = (-u1 / drone_mass_KG) * sin(state_phi.at(0)); // y_ddot

        state_phi.at(3) = phi_c;
        // Attitude controller
        for(int i{0}; i < 10; i++){
            float u2 = pid->turning_moment(state_phi);
            state_phi.at(2) = u2 / pid->get_drone_inertia(); // phi_ddot

            

            simulate_displacement(dt/10., state_z, state_y, state_phi);
        }

        if(step % 10 == 0)
            draw_pid_response(img, step, state_z.at(0), state_y.at(0));

        std::cout<< "Z: " << state_z.at(0) << " Y: " << state_y.at(0) << " Applying Thrust: " << u1 <<std::endl;
    }

    
    cv::Point start = cv::Point(0., 0.);
    circle(img, start, 4, cv::Scalar(255, 0, 0), -1, cv::LINE_8);
    cv::Point goal = cv::Point(y_des, z_des);
    circle(img, goal, 4, cv::Scalar(0, 0, 255), -1, cv::LINE_8);

    cv::namedWindow("Image",cv::WINDOW_AUTOSIZE);
    cv::flip(img, img, 0);
    cv::imshow("Image", img);
    cv::waitKey(0);

    std::cout<< "\n\n";
    return 0;
}

/*
3D State variables
Location : x y z
Orientation : phi theta psi (in terme of euler angles or quaternions or other) => In World frame
Velocity : x_dot y_dot z_dot
Body Rate : p q r (Not the same as phi_dot, theta_dot or z_dot) => Here it's in the Body Frame : Mesured by the gyro of the vehicle

*/