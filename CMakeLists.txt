cmake_minimum_required(VERSION 3.13.0)

# Options
if (CMAKE_CURRENT_SOURCE_DIR STREQUAL CMAKE_SOURCE_DIR)
    option(MOE_EXAMPLES "Turn ON to build example executable(s)" ON)
else()
    option(MOE_EXAMPLES "Turn ON to build example executable(s)" OFF)
endif()

# create project
project(mahiopenexo VERSION 0.1.0 LANGUAGES CXX)

# defines conventional GNU installation directories
include(GNUInstallDirs)

# set compiler flags
if (CMAKE_CXX_COMPILER_ID MATCHES "GNU")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -03") # all warnings
elseif(CMAKE_CXX_COMPILER_ID MATCHES "MSVC")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /W3") # warning level 4
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP") # multicore build
endif()

# Use fetch content to get libraries that moe is dependent on
include(FetchContent)

# MAHI DAQ
FetchContent_Declare(mahi-daq GIT_REPOSITORY https://github.com/mahilab/mahi-daq.git)
FetchContent_MakeAvailable(mahi-daq)

# MAHI ROBO
FetchContent_Declare(mahi-robo GIT_REPOSITORY https://github.com/mahilab/mahi-robo.git)
FetchContent_MakeAvailable(mahi-robo)

# MAHI COM
FetchContent_Declare(mahi-com GIT_REPOSITORY https://github.com/mahilab/mahi-com.git)
FetchContent_MakeAvailable(mahi-com)

# this normally gets set in the mahi-mpc cmakelists, but if you want to test the moe-mpc examples
# here, you need to set this on. Just make sure to not push it to github or else everyone that
# uses moe will have to build casadi
option(WITH_MAHI_MPC "Turn ON to build mahi-mpc" OFF)
if (WITH_MAHI_MPC)
    FetchContent_Declare(mahi-mpc GIT_REPOSITORY https://github.com/mahilab/mahi-mpc.git
                                GIT_TAG origin/main)
    FetchContent_MakeAvailable(mahi-mpc)
endif()
# add definitions
add_definitions(-D_CRT_SECURE_NO_WARNINGS -DNOMINMAX -D_WINSOCK_DEPRECATED_NO_WARNINGS)

file(GLOB_RECURSE INC_MOE "include/*.hpp")

add_library(moe)
add_library(moe::moe ALIAS moe)
set_target_properties(moe PROPERTIES DEBUG_POSTFIX -d)
target_compile_features(moe PUBLIC cxx_std_11)
set_target_properties(moe PROPERTIES OUTPUT_NAME moe)

target_compile_features(moe PUBLIC cxx_std_11)
install(TARGETS moe EXPORT moe-targets LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR})

target_sources(moe PRIVATE ${SRC_MOE} ${INC_MOE})

target_include_directories(moe
PUBLIC
$<INSTALL_INTERFACE:include>
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
PRIVATE
${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_subdirectory(src/MOE/MahiOpenExo)

target_link_libraries(moe mahi::daq mahi::robo mahi::com)
if (WITH_MAHI_MPC)
    target_link_libraries(moe mahi::mpc)
endif()

if(MOE_EXAMPLES)
    message("Building MOE Examples")
    add_subdirectory(examples)
    add_subdirectory(setup)
endif()

# function(link_mpc mpc_target_name)
# add_custom_command(
#     TARGET ${mpc_target_name} POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E copy_if_different
#             ${CMAKE_SOURCE_DIR}/build/_deps/casadi-build/$<CONFIG>/casadi.dll
#             $<TARGET_FILE_DIR:${mpc_target_name}>)     
# add_custom_command(
#     TARGET ${mpc_target_name} POST_BUILD
#     COMMAND ${CMAKE_COMMAND} -E copy_if_different
#             ${CMAKE_SOURCE_DIR}/build/_deps/casadi-build/$<CONFIG>/casadi_nlpsol_ipopt.dll
#             $<TARGET_FILE_DIR:${mpc_target_name}>)     
# endfunction()