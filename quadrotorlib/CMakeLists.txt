add_library(quadrotorlib
src/quadrotor.cpp
src/attitude_controller.cpp
src/position_controller.cpp)

target_include_directories(quadrotorlib PUBLIC include)