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
    circle(img, p, 2, cv::Scalar(0, 255, 0), 1, cv::LINE_8);
}

std::array<float, 3> simulate_displacement(float gravity, float drone_mass_KG, float dt, float thrust, float z_dot, float z){

    float z_ddot {(thrust / drone_mass_KG) - gravity};
    float delta_z_dot {z_ddot * dt};
    z_dot += delta_z_dot;

    float delta_z {z_dot * dt};
    z += delta_z;

    std::array<float, 3> state = {z, delta_z_dot, z_ddot};

    return state;
}


int main(){
    float gravity {9.81};
    float drone_mass_KG{0.18};
    float min_motor_thrust_N{drone_mass_KG * (gravity / 2.0)};
    float max_motor_thrust_N{2 * drone_mass_KG * gravity};

    unique_ptr<PID_Controller> pid = make_unique<PID_Controller>(min_motor_thrust_N, max_motor_thrust_N, drone_mass_KG);

    // Init sensors
    std::shared_ptr<float> z = std::make_shared<float>(0.0);
    std::shared_ptr<float> z_dot = std::make_shared<float>(0.0);
    std::shared_ptr<float> ff_z_ddot = std::make_shared<float>(0.0);

    std::shared_ptr<float> z_ptr {z};
    std::shared_ptr<float> z_dot_ptr {z_dot};
    std::shared_ptr<float> ff_z_ddot_ptr {ff_z_ddot};

    // trajectory planner
    float z_des {10.0};
    float z_dot_des {0.5};
    float dt {.05};

    for(int step{0}; step < 1000; step++){

        float u = pid->thrust(*z, z_des, z_dot_des, *z_dot, *ff_z_ddot);
        
        //update
        std::array<float, 3> state = simulate_displacement(gravity, drone_mass_KG, dt, u, *z_dot, *z);
        *z_ptr = state.at(0);
        *z_dot_ptr = state.at(1);
        *ff_z_ddot_ptr = state.at(2);

        std::cout << *z <<std::endl;
    }

    
    std::cout<< "\n\n";
    return 0;
}