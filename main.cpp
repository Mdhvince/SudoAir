#include <iostream>
#include <memory>
#include <array>

#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include "controllib/include/controllib/pid_controller.h"

using std::unique_ptr;
using std::make_unique;

void draw_pid_response(cv::Mat img, int step, float z){
    cv::Point p = cv::Point(step*step, z);
    circle(img, p, 2, cv::Scalar(0, 255, 0), 1, cv::LINE_8);
}

std::array<float, 2> update_state_z(float dt, float acceleration_z, float velocity_z, float position_z){

    // float net_TorqueM {5.0}; // on z
    // float drone_M_Inertia {2.0}; // on z

    // float ang_velocity_z {0.0};
    // float ang_position_z {0.0};

    //update velocity from aceleration (integral)
    //update position from velocity (second integral)

    float delta_velocity_z {acceleration_z * dt};
    velocity_z += delta_velocity_z;
    
    float delta_position {velocity_z * dt};
    position_z += delta_position;

    // float ang_acceleration_z {net_TorqueM / drone_M_Inertia};
    // float delta_yaw_velocity {ang_acceleration_z * dt};
    // ang_velocity_z += delta_yaw_velocity;
    // float delta_yaw_position {ang_velocity_z * dt};
    // ang_position_z += delta_yaw_position;

    std::array<float, 2> s {delta_velocity_z, position_z};
    return s;

}

int main(){
    float max_motor_thrust_N = 0.58;
    float drone_mass_KG = 0.027;
    unique_ptr<PID_Controller> pid = make_unique<PID_Controller>(max_motor_thrust_N, drone_mass_KG);
    
    std::shared_ptr<float> z { new float(0.0) };
    float z_des=250.0;
    float vel_z_des=0.20;
    std::shared_ptr<float> vel_z { new float(0.0) };
    float dt=1.0;

    std::shared_ptr<float> z_ptr {z};
    std::shared_ptr<float> vel_z_ptr {vel_z};

    
    cv::Mat img = cv::Mat::zeros(500, 1000, CV_8UC3);
    cv::line(img, cv::Point(0, z_des), cv::Point(1000, z_des), cv::Scalar(0, 0, 255), 1, cv::LINE_8);

    for(int step{0}; step <= 100; step++){
        float thrust = pid->altitude_control(*z, z_des, vel_z_des, *vel_z, dt);
        float acc = pid->get_acceleration_z();
        
        std::array<float, 2> s = update_state_z(dt, acc, *vel_z, *z);
        *z_ptr = s.at(1);
        *vel_z_ptr = s.at(0);

        draw_pid_response(img, step, *z);
    }

    cv::namedWindow("Display", cv::WINDOW_NORMAL);
    cv::flip(img, img, 0);
    cv::imshow("Display", img);
    int k = cv::waitKey(0);
    
    std::cout<< "\n\n";
    return 0;
}