#ifndef _QUADROTOR_H_
#define _QUADROTOR_H_
#include <iostream>
#include <array>

#include "pid_controller.h"


class QuadRotor{
    PID_Controller pid;
    std::array<float, 4> quad_specs; // {drone_mass, max_motor_thrust, min_motor_thrust, I_xx}

private:

    QuadRotor();
    QuadRotor(std::array<float, 4> quad_specs,
              std::array<float, 5> &state_z, std::array<float, 5> &state_y, std::array<float, 5> &state_phi);
    QuadRotor(const QuadRotor &source);
    ~QuadRotor();

};

#endif