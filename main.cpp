#include <iostream>
#include <memory>
#include <array>
#include <algorithm>
#include <unistd.h>
#include "controllib/include/controllib/pid_controller.h"

using std::unique_ptr;
using std::make_unique;


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
    unsigned int microsecond = 1000000;
    float max_motor_thrust_N = 0.58;
    float drone_mass_KG = 0.027;
    unique_ptr<PID_Controller> pid = make_unique<PID_Controller>(max_motor_thrust_N, drone_mass_KG);
    
    std::shared_ptr<float> z { new float(0.0) };
    float z_des=10.0;
    float vel_z_des=0.10;
    std::shared_ptr<float> vel_z { new float(0.0) };
    float dt=0.5;


    std::shared_ptr<float> z_ptr {z};
    std::shared_ptr<float> vel_z_ptr {vel_z};

    for(int step{0}; step <= 100; step++){
        float thrust = pid->altitude_control(*z, z_des, vel_z_des, *vel_z, dt);
        float acc = pid->get_acceleration_z();

        std::cout<< step <<std::endl;
        std::cout<<"Pos: " << *z <<std::endl;
        std::cout<<"New Velocity: " << acc * dt <<std::endl;
        std::cout<<"New Thrust(N): " << thrust <<std::endl;
        std::cout<<"Thrust/Weight ratio: " << (thrust*0.101)/drone_mass_KG <<std::endl;
        
        std::array<float, 2> s = update_state_z(dt, acc, *vel_z, *z);
        *z_ptr = s.at(1);
        *vel_z_ptr = s.at(0);
        // usleep(0.1 * microsecond);
        std::cout<< "\n";
    }
    
    std::cout<< "\n\n";
    return 0;
}