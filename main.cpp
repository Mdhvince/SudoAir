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


int main(){
    float max_motor_thrust_N{0.58};
    float drone_mass_KG{0.027};
    unique_ptr<PID_Controller> pid = make_unique<PID_Controller>(max_motor_thrust_N, drone_mass_KG);

    float z {0};
    float vel_z {0};
    float z_des {1};
    float vel_z_des {0};
    float ff_z_ddot {0};
    
    float u = pid->thrust(z, z_des, vel_z_des, vel_z, ff_z_ddot);

    std::cout << u;

    
    std::cout<< "\n\n";
    return 0;
}