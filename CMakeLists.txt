cmake_minimum_required(VERSION 3.5)

project(SudoAir VERSION 1.0.0)

add_definitions(-std=c++14)
set(CXX_FLAGS "-Wall")

#ControlLib
add_subdirectory(controllib)

#Libtorch
# find_package(Torch REQUIRED)
# set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${TORCH_CXX_FLAGS}")

set(EXECUTABLE app)
add_executable(${EXECUTABLE} main.cpp)

target_link_libraries(${EXECUTABLE} PRIVATE controllib)
# target_link_libraries(${EXECUTABLE} PRIVATE controllib "${TORCH_LIBRARIES}")