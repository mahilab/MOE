add_executable(jeffTest jeffTest.cpp)
target_link_libraries(jeffTest moe::moe)

add_executable(rom_demo ex_rom_demo.cpp)
target_link_libraries(rom_demo moe::moe)

add_executable(single_dof_testing ex_single_dof_testing.cpp)
target_link_libraries(single_dof_testing moe::moe)

add_executable(ex_rom_CSU ex_rom_CSU.cpp)
target_link_libraries(ex_rom_CSU moe::moe)

add_executable(grav_comp ex_grav_comp.cpp)
target_link_libraries(grav_comp moe::moe)

add_executable(arm_prop_data_collection ex_arm_prop_data_collection.cpp)
target_link_libraries(arm_prop_data_collection moe::moe)

if(WITH_MAHI_MPC)
    add_executable(mpc_gen ex_mpc_gen.cpp)
    target_link_libraries(mpc_gen mahi::mpc moe::moe)
    link_mpc(mpc_gen)

    add_executable(mpc_gen_one_dof ex_mpc_gen_one_dof.cpp)
    target_link_libraries(mpc_gen_one_dof mahi::mpc moe::moe)
    link_mpc(mpc_gen_one_dof)
    
    add_executable(mpc_demo ex_mpc_demo.cpp)
    target_link_libraries(mpc_demo mahi::mpc moe::moe)
    link_mpc(mpc_demo)

    add_executable(mpc_demo_one_dof ex_mpc_demo_one_dof.cpp)
    target_link_libraries(mpc_demo_one_dof mahi::mpc moe::moe)
    link_mpc(mpc_demo_one_dof)
endif()