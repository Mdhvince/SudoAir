#include <iostream>
#include <memory>
#include <array>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "quadrotorlib/include/quadrotorlib/pid_controller.h"
#include "quadrotorlib/include/quadrotorlib/quadrotor.h"

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

void simulate_displacement(float dt, std::array<float, 9> &xyz_state, std::array<float, 9> &angle_state_wf){
    
    if(xyz_state.at(8) != 0.0){
        float delta_z_dot {xyz_state.at(8) * dt};
        xyz_state.at(5) = delta_z_dot;
        float delta_pos_z {xyz_state.at(5) * dt};
        xyz_state.at(2) += delta_pos_z;
    }
    if(xyz_state.at(7) != 0.0){
        float delta_y_dot {xyz_state.at(7) * dt};
        xyz_state.at(4) = delta_y_dot;
        float delta_pos_y {xyz_state.at(4) * dt};
        xyz_state.at(1) += delta_pos_y;
    }
    if(angle_state_wf.at(6) != 0.0){
        float delta_phi_dot {angle_state_wf.at(6) * dt};
        angle_state_wf.at(3) = delta_phi_dot;
        float delta_phi {angle_state_wf.at(3) * dt};
        angle_state_wf.at(0) += delta_phi;
    }
}


int main(){
    float gravity {9.81};
    float drone_mass_KG{0.027};
    float Ixx {2.3951e-5};
    float max_motor_thrust_N{0.5687857};
    float min_motor_thrust_N{(drone_mass_KG * gravity) - .1};

    // From trajectory planner
    float x_des {50.0};
    float y_des {800.0};
    float z_des {300.0};
    float x_dot_des {1.5};
    float z_dot_des {1.5};
    float y_dot_des {1.5};
    
    float dt {1.};

    unique_ptr<PID_Controller> pid = make_unique<PID_Controller>();

    std::array<float, 9> xyz_state {0.0, 50.0, 290.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<float, 9> xyz_state_des {x_des, y_des, z_des, x_dot_des, y_dot_des, z_dot_des, 0.0, 0.0, 0.0};
    std::array<float, 9> angle_state_wf {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    std::array<float, 9> angle_state_wf_des {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0};

    cv::Mat img = cv::Mat::zeros(cv::Size(1500, 700), CV_8UC3);
    cv::Point start = cv::Point(xyz_state.at(1), xyz_state.at(2));
    circle(img, start, 15, cv::Scalar(255, 0, 0), 1, cv::LINE_8);
    cv::Point goal = cv::Point(y_des, z_des);
    circle(img, goal, 15, cv::Scalar(0, 0, 255), 1, cv::LINE_8);

    

    for(int step{0}; step < 10000; step++){
        
        

        float u1 = pid->control_altitude(xyz_state, xyz_state_des, angle_state_wf, gravity, drone_mass_KG, min_motor_thrust_N, max_motor_thrust_N);
        
        angle_state_wf_des.at(0) = pid->control_lateral(xyz_state, xyz_state_des, u1, drone_mass_KG);      // phi_cmd (requiered roll angle to move the quadrotor : in the world frame)
        xyz_state.at(8) = ((u1 * cos(angle_state_wf.at(0))) / drone_mass_KG) - gravity;                    // z_ddot_target
        xyz_state.at(7) = ((-u1 * sin(angle_state_wf.at(0))) / drone_mass_KG);                             // y_ddot_target
        

        // Attitude controller
        for(int i{0}; i < 10; i++){
            float u2 = pid->control_attitude(angle_state_wf, angle_state_wf_des, Ixx);
            angle_state_wf.at(6) = u2 / Ixx; // phi_ddot
            simulate_displacement(dt/10., xyz_state, angle_state_wf);
        }

        if(step % 50 == 0)
            draw_pid_response(img, step, xyz_state.at(2), xyz_state.at(1));
        

        // std::cout<< "Z: " << xyz_state.at(2) << " Y: " << xyz_state.at(1) <<std::endl;
        // std::cout<< "Velocity z: " << xyz_state.at(5) << " Velocity y: " << xyz_state.at(4) <<std::endl;
        // std::cout<< "Acc z: " << xyz_state.at(8) << " Acc y: " << xyz_state.at(7) <<std::endl;
        std::cout<<"(%) Thrust Applied: " << ((u1/max_motor_thrust_N)*100) <<" %"<<std::endl;
        std::cout<<"\n";

        // change trajectory
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

/*
3D State variables
Location : x y z
Orientation : phi theta psi (in terme of euler angles or quaternions or other) => In World frame
Velocity : x_dot y_dot z_dot
Body Rate : p q r (Not the same as phi_dot, theta_dot or z_dot) => Here it's in the Body Frame : Mesured by the gyro of the vehicle

*/