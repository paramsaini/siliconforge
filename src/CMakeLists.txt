add_library(siliconforge_lib STATIC
    # ── Core ──────────────────────────────────────────────────────────
    core/aig.cpp
    core/netlist.cpp
    core/design_db.cpp
    core/hierarchy.cpp
    core/die_to_die.cpp
    core/liberty_parser.cpp
    core/liberty_cache.cpp
    core/name_pool.cpp
    core/def_parser.cpp
    core/lef_parser.cpp
    core/lef_writer.cpp
    core/spef_parser.cpp
    core/parasitic_extract.cpp
    core/report_gen.cpp

    # ── Frontend Parsers ──────────────────────────────────────────────
    frontend/verilog_parser.cpp
    frontend/sdc_parser.cpp
    frontend/sva_parser.cpp
    frontend/upf_parser.cpp
    frontend/vhdl_parser.cpp
    frontend/sv_elaborate.cpp
    frontend/sv_parser.cpp

    # ── Synthesis ─────────────────────────────────────────────────────
    synth/aig_opt.cpp
    synth/datapath.cpp
    synth/behavioral_synth.cpp
    synth/clock_gating.cpp
    synth/eco.cpp
    synth/fsm_extract.cpp
    synth/multi_vt.cpp
    synth/multibit.cpp
    synth/resource_share.cpp
    synth/retiming.cpp
    synth/tech_mapper.cpp
    synth/npn_match.cpp

    # ── SAT Solver ────────────────────────────────────────────────────
    sat/cdcl_solver.cpp

    # ── Formal Verification ───────────────────────────────────────────
    formal/tseitin.cpp
    formal/equiv_checker.cpp
    formal/bmc.cpp
    formal/k_induction.cpp
    formal/lec.cpp
    formal/sva_engine.cpp
    formal/advanced_formal.cpp

    # ── Simulation ────────────────────────────────────────────────────
    sim/simulator.cpp

    # ── Lint ──────────────────────────────────────────────────────────
    lint/lint_engine.cpp

    # ── DFT ───────────────────────────────────────────────────────────
    dft/podem.cpp
    dft/fault_sim.cpp
    dft/scan_insert.cpp
    dft/scan_compress.cpp
    dft/jtag_bist.cpp

    # ── HLS ───────────────────────────────────────────────────────────
    hls/c_parser.cpp

    # ── Macro Generation ──────────────────────────────────────────────
    macro/memory_compiler.cpp

    # ── Place & Route ─────────────────────────────────────────────────
    pnr/physical.cpp
    pnr/floorplan.cpp
    pnr/placer.cpp
    pnr/global_router.cpp
    pnr/detailed_router.cpp
    pnr/detailed_router_v2.cpp
    pnr/distributed_route.cpp
    pnr/cts.cpp
    pnr/post_route_opt.cpp
    pnr/timing_closure.cpp
    pnr/gdsii_writer.cpp
    pnr/oasis_writer.cpp
    pnr/chip_assembler.cpp
    pnr/tsv_mgr.cpp
    pnr/ai_tuner.cpp
    pnr/power_plan.cpp
    pnr/metal_fill.cpp
    pnr/cell_insert.cpp
    pnr/detailed_placer.cpp
    pnr/multi_pattern.cpp
    pnr/gdsii_reader.cpp

    # ── Timing & Power ────────────────────────────────────────────────
    timing/sta.cpp
    timing/ssta.cpp
    timing/power.cpp
    timing/parasitics.cpp
    timing/ccs_delay.cpp
    timing/ir_drop.cpp
    timing/pdn.cpp
    timing/noise.cpp
    timing/signal_integrity.cpp
    timing/electromigration.cpp
    timing/mcmm.cpp
    timing/thermal.cpp
    timing/sdf_writer.cpp

    # ── Physical Verification ─────────────────────────────────────────
    verify/drc.cpp
    verify/lvs.cpp
    verify/cdc.cpp
    verify/antenna.cpp
    verify/opc.cpp
    verify/reliability.cpp
    verify/erc.cpp
    verify/esd.cpp
    verify/latchup.cpp
    verify/rdc.cpp

    # ── ML Models ─────────────────────────────────────────────────────
    ml/congestion_model.cpp
    ml/gnn.cpp
    ml/timing_model.cpp
    ml/ml_opt.cpp

    # ── Standard Cell ─────────────────────────────────────────────────
    stdcell/cell_layout.cpp
    stdcell/cell_drc.cpp
    stdcell/cell_lvs.cpp
    stdcell/characterizer.cpp

    # ── Visualization ─────────────────────────────────────────────────
    viz/dashboard.cpp
    viz/html_export.cpp
    viz/logic_viz.cpp
    viz/rtl_viz.cpp
    viz/sim_viz.cpp

    # ── SPICE Engine ────────────────────────────────────────────────────
    spice/spice_parser.cpp
    spice/spice_engine.cpp

    # ── Flow Engine & Shell ───────────────────────────────────────────
    flow/engine.cpp
    shell/sf_shell.cpp
    shell/tcl_interp.cpp
)

target_include_directories(siliconforge_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(siliconforge_lib PUBLIC Threads::Threads)
if(OpenMP_CXX_FOUND)
    target_link_libraries(siliconforge_lib PUBLIC OpenMP::OpenMP_CXX)
    target_compile_definitions(siliconforge_lib PUBLIC SF_HAS_OPENMP=1)
endif()

add_executable(siliconforge main.cpp)
target_link_libraries(siliconforge PRIVATE siliconforge_lib)
