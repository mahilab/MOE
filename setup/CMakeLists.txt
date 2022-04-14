add_executable(encoder_test encoder_test.cpp common.hpp)
target_link_libraries(encoder_test moe)
target_include_directories(encoder_test PRIVATE ${PROJECT_SOURCE_DIR}/setup)

add_executable(motor_test motor_test.cpp common.hpp)
target_link_libraries(motor_test moe)
target_include_directories(motor_test PRIVATE ${PROJECT_SOURCE_DIR}/setup)

add_executable(pd_controller_test pd_controller_test.cpp common.hpp)
target_link_libraries(pd_controller_test moe)
target_include_directories(pd_controller_test PRIVATE ${PROJECT_SOURCE_DIR}/setup)