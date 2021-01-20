cmake_minimum_required(VERSION 3.5)

project(SudoAir VERSION 1.0.0)

add_definitions(-std=c++14)
set(CXX_FLAGS "-Wall")

#ControlLib
add_subdirectory(controllib)

#OpenCV
find_package(OpenCV REQUIRED)

message(STATUS "OpenCV library status:")
message(STATUS "    config: ${OpenCV_DIR}")
message(STATUS "    version: ${OpenCV_VERSION}")
message(STATUS "    libraries: ${OpenCV_LIBS}")
message(STATUS "    include path: ${OpenCV_INCLUDE_DIRS}")

set(EXECUTABLE app)
add_executable(${EXECUTABLE} main.cpp)

target_link_libraries(${EXECUTABLE} PRIVATE controllib ${OpenCV_LIBS})