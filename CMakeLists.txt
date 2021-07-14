# initialize required cmake options
cmake_minimum_required(VERSION 3.13)
project(experiment_project)

# get the relevant libraries to use from github. FetchContent is great because it pulls the most recent version of a library and will add it to your project.
include(FetchContent)
FetchContent_Declare(mahi-daq GIT_REPOSITORY https://github.com/mahilab/mahi-daq.git) 
FetchContent_MakeAvailable(mahi-daq)

FetchContent_Declare(mahi-robo GIT_REPOSITORY https://github.com/mahilab/mahi-robo.git) 
FetchContent_MakeAvailable(mahi-robo)

include_directories(src)

add_executable(encoder_test src/encoder_test.cpp src/common.hpp)
target_link_libraries(encoder_test mahi::daq)

add_executable(motor_test src/motor_test.cpp src/common.hpp)
target_link_libraries(motor_test mahi::daq)

add_executable(pd_controller_test src/pd_controller_test.cpp src/common.hpp)
target_link_libraries(pd_controller_test mahi::daq mahi::robo)

add_executable(moe_test src/moe_test.cpp src/common.hpp)
target_link_libraries(moe_test mahi::daq mahi::robo)