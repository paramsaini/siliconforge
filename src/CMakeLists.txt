add_library(siliconforge_lib STATIC
    core/aig.cpp
    core/netlist.cpp
    sat/cdcl_solver.cpp
    formal/tseitin.cpp
    formal/equiv_checker.cpp
    formal/bmc.cpp
    formal/k_induction.cpp
    sim/simulator.cpp
    lint/lint_engine.cpp
    dft/podem.cpp
    dft/fault_sim.cpp
)

target_include_directories(siliconforge_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(siliconforge main.cpp)
target_link_libraries(siliconforge PRIVATE siliconforge_lib)
