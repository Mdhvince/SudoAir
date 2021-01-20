add_library(controllib
src/pid_controller.cpp)

target_include_directories(controllib PUBLIC include)