add_executable(jeffTest jeffTest.cpp)
target_link_libraries(jeffTest moe::moe)

add_executable(rom_demo ex_rom_demo.cpp)
target_link_libraries(rom_demo moe::moe)

add_executable(single_dof_testing ex_single_dof_testing.cpp)
target_link_libraries(single_dof_testing moe::moe)