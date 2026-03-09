add_executable(test_phase1 test_phase1.cpp)
target_link_libraries(test_phase1 PRIVATE siliconforge_lib)
add_test(NAME Phase1Tests COMMAND test_phase1)

add_executable(test_phase2 test_phase2.cpp)
target_link_libraries(test_phase2 PRIVATE siliconforge_lib)
add_test(NAME Phase2Tests COMMAND test_phase2)
