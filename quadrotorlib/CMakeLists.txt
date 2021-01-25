add_library(quadrotorlib
src/quadrotor.cpp
src/pid_controller.cpp)

target_include_directories(quadrotorlib PUBLIC include)