add_executable(jeffTest jeffTest.cpp)
target_link_libraries(jeffTest moe::moe)

add_executable(rom_demo ex_rom_demo.cpp)
target_link_libraries(rom_demo moe::moe)