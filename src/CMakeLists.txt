add_library(siliconforge_lib STATIC
    # ── Core ──────────────────────────────────────────────────────────
    core/aig.cpp
    core/netlist.cpp
    core/design_db.cpp
    core/hierarchy.cpp
    core/die_to_die.cpp
    core/liberty_parser.cpp
    core/def_parser.cpp
    core/lef_parser.cpp
    core/lef_writer.cpp
    core/report_gen.cpp

    # ── Frontend Parsers ──────────────────────────────────────────────
    frontend/verilog_parser.cpp
    frontend/sdc_parser.cpp
    frontend/sva_parser.cpp
    frontend/upf_parser.cpp

    # ── Synthesis ─────────────────────────────────────────────────────
    synth/aig_opt.cpp
    synth/behavioral_synth.cpp
    synth/clock_gating.cpp
    synth/eco.cpp
    synth/fsm_extract.cpp
    synth/multi_vt.cpp
    synth/multibit.cpp
    synth/resource_share.cpp
    synth/retiming.cpp
    synth/tech_mapper.cpp

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
    pnr/cts.cpp
    pnr/post_route_opt.cpp
    pnr/gdsii_writer.cpp
    pnr/oasis_writer.cpp
    pnr/chip_assembler.cpp
    pnr/tsv_mgr.cpp
    pnr/ai_tuner.cpp
    pnr/power_plan.cpp
    pnr/metal_fill.cpp
    pnr/cell_insert.cpp

    # ── Timing & Power ────────────────────────────────────────────────
    timing/sta.cpp
    timing/ssta.cpp
    timing/power.cpp
    timing/parasitics.cpp
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
    ml/timing_model.cpp
    ml/ml_opt.cpp

    # ── Visualization ─────────────────────────────────────────────────
    viz/dashboard.cpp
    viz/html_export.cpp
    viz/logic_viz.cpp
    viz/rtl_viz.cpp
    viz/sim_viz.cpp

    # ── Flow Engine & Shell ───────────────────────────────────────────
    flow/engine.cpp
    shell/sf_shell.cpp
    shell/tcl_interp.cpp
)

target_include_directories(siliconforge_lib PUBLIC ${CMAKE_CURRENT_SOURCE_DIR})

add_executable(siliconforge main.cpp)
target_link_libraries(siliconforge PRIVATE siliconforge_lib)
