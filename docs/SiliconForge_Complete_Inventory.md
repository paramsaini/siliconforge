# SiliconForge RTL-to-GDSII Flow — Complete Inventory

---

## 1. FRONTEND SUBSYSTEM

### 1.1 Verilog Parser (`verilog_parser.hpp`)

**File Path**: `frontend/verilog_parser.hpp`

**Structures & Enums**:
- `VerilogParseResult` — Parse result struct with success flag, module info, counts, error message
- `Token::Type` enum — 70+ token types (IDENT, NUMBER, LPAREN, ALWAYS, IF, etc.)
- `ModuleDef` — Module definition cache with tokens, port names, directions, ranges

**Key Classes**:
- `VerilogParser` — Main parser supporting structural + behavioral Verilog

**Public API**:
- `parse_string(src, nl) → VerilogParseResult`
- `parse_file(filename, nl) → VerilogParseResult`
- `to_verilog(nl, module_name) → std::string`

**Algorithms & Key Features**:
- **Precedence-based recursive descent expression parser** (handles 12-level operator precedence)
- **Structural hashing** for gate instantiation
- **Behavioral synthesis integration** via `BehavioralAST`
- **Module hierarchical elaboration** with MAX_HIERARCHY_DEPTH=16
- **Parameter evaluation** with `eval_const_expr()`
- **Generate block parsing** (generate/genvar/generate_for/generate_if)
- **Function/Task parsing** (FUNCTION_KW, TASK_KW, ENDFUNCTION_KW, ENDTASK_KW)
- **Memory array support** (multi-dimensional arrays, word_width, depth)
- **Signed signal tracking** via `signed_signals_` set

**Key Thresholds**:
- `MAX_HIERARCHY_DEPTH = 16` — Maximum module instantiation nesting

---

### 1.2 SDC Parser (`sdc_parser.hpp`)

**File Path**: `frontend/sdc_parser.hpp`

**Structures**:
- `SdcClock` — Clock definition (name, port, period_ns, waveform_rise/fall, uncertainty)
- `SdcInputDelay` — Input delay constraint (port, clock, delay_ns, is_max)
- `SdcOutputDelay` — Output delay constraint (port, clock, delay_ns, is_max)
- `SdcException` — Path exception (type: FALSE_PATH|MULTICYCLE_PATH|MAX_DELAY|MIN_DELAY, from, to, value, multiplier)
- `SdcConstraints` — Container with clocks, input_delays, output_delays, exceptions, fanout/transition/capacitance maps

**Public API**:
- `parse_string(src, sdc) → SdcParseResult`
- `parse_file(filename, sdc) → SdcParseResult`
- `to_sdc(constraints) → std::string`

**Key Methods**:
- `SdcConstraints::find_clock(name) → const SdcClock*`
- `SdcConstraints::get_clock_period(name) → double`

---

### 1.3 SVA Parser (`sva_parser.hpp`)

**File Path**: `frontend/sva_parser.hpp`

**Enums & Structures**:
- `SvaOp` enum — PROP_OVERLAPPING (`|->`) | PROP_NON_OVERLAPPING (`|=>`) | LITERAL
- `SvaNode` — LTL expression node (op, literal, left, right)
- `SvaProperty` — Property definition (name, clock_domain, expr, is_assert)

**Public API**:
- `parse(source) → std::vector<SvaProperty>`

**Algorithms**:
- **Linear Temporal Logic (LTL) property parsing** with overlapping/non-overlapping implication operators
- **Tokenization** with `tokenize()`
- **Recursive expression parser** with `parse_expr()`

---

### 1.4 UPF Parser (`upf_parser.hpp`)

**File Path**: `frontend/upf_parser.hpp`

**Structures** (IEEE 1801 compliant):
- `SupplyNet` — Supply definition (name, voltage, is_resolved, connected_supply_port)
- `SupplyPort` — Supply port with direction (IN|OUT) and domain
- `PowerDomain` — Power domain hierarchy (name, parent, elements, primary power/ground nets, voltage)
- `IsolationStrategy` — Isolation cell insertion (domain, type: CLAMP_0|CLAMP_1|LATCH|AND|OR, applies_to, signal, sense: HIGH|LOW, location: self|parent|fanout)
- `LevelShifter` — Level shifter definition (domain, applies_to, rule: low_to_high|high_to_low|both, input/output_voltage)
- `RetentionStrategy` — Retention logic (domain, save/restore signals, condition: EDGE|LEVEL)
- `PowerState` — Power state (name, supply_voltages map, simstate: NORMAL|CORRUPT|RESET|HOLD)
- `PowerStateTable` — Multi-state table

**Key Classes**:
- `UpfParser` — Parser with TCL-like tokenization
- `UpfChecker` — DRC checker for power intent consistency

**Public API**:
- `parse_file(filename) → UpfDesign`
- `parse_string(content) → UpfDesign`
- `UpfDesign::validate() → bool`
- `UpfDesign::to_upf() → std::string`
- `UpfDesign::report() → std::string`

**Algorithms**:
- **TCL command parsing** with backslash continuation, braces, quoted strings
- **Flag extraction** (`get_flag()`, `get_flag_list()`, `has_flag()`)
- **Per-command parsers** (create_power_domain, create_supply_net, set_isolation, set_level_shifter, set_retention, add_power_state, create_pst, add_pst_state)

---

## 2. CORE SUBSYSTEM

### 2.1 Types (`core/types.hpp`)

**File Path**: `core/types.hpp`

**Enums & Classes**:
- `Logic4` enum (uint8_t) — ZERO(0) | ONE(1) | X(2) | Z(3) — IEEE 1364/1800 4-state logic

**Inline Truth Table Functions** (IEEE 1364-2005):
- `logic_not(a) → Logic4`
- `logic_and(a, b) → Logic4` — 0 dominates, then X, then Z
- `logic_or(a, b) → Logic4` — 1 dominates, then X, then Z
- `logic_xor(a, b) → Logic4`
- `logic_to_char(v) → char`
- `char_to_logic(c) → Logic4`

**Key Class**: `BitVector`
- **LSB-first storage** (bits_[0] = LSB)
- **Constructor**: `BitVector(width, init=Logic4::X)`
- **Factory**: `from_uint(width, value) → BitVector`
- **Accessors**: `operator[](i) → Logic4`, `width() → size_t`
- **Query**: `is_known() → bool`, `to_uint() → uint64_t`, `to_string() → std::string`

---

### 2.2 Netlist (`core/netlist.hpp`)

**File Path**: `core/netlist.hpp`

**Types**:
- `NetId = int32_t` — Wire identifier
- `GateId = int32_t` — Gate instance identifier

**Enums**:
- `GateType` — INPUT, OUTPUT, BUF, NOT, AND, OR, NAND, NOR, XOR, XNOR, MUX, DFF, DLATCH, TRI, CONST0, CONST1, BUFIF0, BUFIF1, NOTIF0, NOTIF1

**Structures**:
- `Net` — Wire (id, name, Logic4 value, next_value, fanout gates, driver gate)
- `Gate` — Logic cell (id, type, name, inputs, output, clk, reset, init_val for DFF)

**Key Class**: `Netlist`
- **Add operations**: `add_net(name) → NetId`, `add_gate(type, inputs, output, name) → GateId`, `add_dff(d, clk, q, reset, name) → GateId`
- **Mark as I/O**: `mark_input(id)`, `mark_output(id)`
- **Accessors**: `net(id) → Net&`, `gate(id) → Gate&`, `num_nets() → size_t`, `num_gates() → size_t`
- **Collections**: `primary_inputs() → const std::vector<NetId>&`, `primary_outputs()`, `flip_flops()`, `nets()`, `gates()`
- **Analysis**: `topo_order() → std::vector<GateId>` — Topological sort of combinational gates
- **Evaluation**: `eval_gate(type, inputs) → Logic4` — Static gate evaluation
- **Reporting**: `print_stats()`, `clear()`

---

### 2.3 AIG (`core/aig.hpp`)

**File Path**: `core/aig.hpp`

**Types & Constants**:
- `AigLit = uint32_t` — AIGER standard encoding: (variable << 1) | complement
- `AIG_FALSE = 0`, `AIG_TRUE = 1`
- **Inline functions**: `aig_var(lit)`, `aig_sign(lit)`, `aig_not(lit)`, `aig_regular(lit)`, `aig_make(var, neg)`

**Structures**:
- `AigNode` — AND gate definition (fanin0, fanin1 as AigLit)
- `AigLatch` — Sequential latch (next, init as AigLit, name)

**Key Class**: `AigGraph`
- **Construction**: `create_input(name) → AigLit`, `create_and(a, b) → AigLit`, `create_or(a, b)`, `create_xor(a, b)`, `create_mux(sel, t, e)`, `create_nand(a, b)`, `create_nor(a, b)`
- **Outputs/Latches**: `add_output(lit, name)`, `add_latch(next, init, name)`
- **Query**: `num_inputs() → size_t`, `num_outputs()`, `num_ands()`, `num_latches()`, `max_var() → uint32_t`
- **Accessors**: `inputs()`, `outputs()`, `latches()`, `input_names()`, `output_names()`
- **Type checks**: `is_const(lit) → bool`, `is_input(var) → bool`, `is_and(var) → bool`, `is_latch(var) → bool`
- **Node access**: `and_node(var) → const AigNode&`, `and_node_mut(var) → AigNode&`
- **Simulation**: `evaluate(input_values) → std::vector<bool>`, `eval_lit(lit, node_vals) → bool`
- **Debug**: `print_stats()`, `to_dot() → std::string`

**Internal Features**:
- **Structural hashing** via `strash_` map for DAG construction
- **Hash key**: `strash_key(a, b) → uint64_t` — Unique key for (min, max) literal pair

**Multi-Bit Class**: `AigVec`
- **Factory**: `create_input(g, w, prefix) → AigVec`
- **Bitwise ops**: `bitwise_and(g, a, b)`, `bitwise_or()`, `bitwise_xor()`, `bitwise_not()`
- **Comparison**: `eq(g, a, b) → AigLit`
- **Arithmetic**: `add(g, a, b, cin) → AigVec`
- **MUX**: `mux(g, sel, t, e) → AigVec`

---

### 2.4 Liberty Parser (`core/liberty_parser.hpp`)

**File Path**: `core/liberty_parser.hpp`

**Structures**:
- `LibertyPin` — Pin model (name, direction: input|output|inout, function, capacitance, max_transition)
- `LibertyTiming` — Timing arc (related_pin, timing_type, cell_rise/fall, rise_transition/fall_transition)
  - **NLDM Tables** (2D lookup): `NldmTable` with index_1 (slew), index_2 (load), values (2D interpolation)
    - `nldm_rise` / `nldm_fall` — Cell delay tables
    - `nldm_rise_tr` / `nldm_fall_tr` — Transition time tables
- `LibertyCell` — Cell definition (name, area, leakage_power, pins, timings)
- `LibertyLibrary` — Technology library (name, technology, nom_voltage, nom_temperature, time_unit, cap_unit, cells)

**Public API**:
- `parse(filename) → bool`
- `parse_string(content) → bool`
- `find_cell(name) → const LibertyCell*`
- `cells_by_function(func) → std::vector<const LibertyCell*>`
- `print_stats()`

**Internal**:
- Tokenizer with Type enum: IDENT, STRING, NUMBER, LBRACE, RBRACE, LPAREN, RPAREN, COLON, SEMI, COMMA, END
- Recursive descent parser: `parse_tokens()`, `parse_group()`, `parse_cell()`, `parse_pin()`, `parse_timing()`, `skip_group()`

---

### 2.5 LEF Parser (`core/lef_parser.hpp`)

**File Path**: `core/lef_parser.hpp`

**Structures** (LEF 5.8 standard):
- `LefUnits` — Unit conversion (database_microns, capacitance_pf, resistance_kohm, time_ns, power_mw)
- `LefLayer` — Technology layer (name, type: ROUTING|CUT|MASTERSLICE|OVERLAP|IMPLANT, layer_num, width/min_width/max_width/pitch/offset/spacing, resistivity, capacitance, direction: HORIZONTAL|VERTICAL|NONE, spacing_table)
- `LefVia` — Via definition with ViaLayer (layer_name, rects), resistance
- `LefViaRule` — Via generation rule (name, is_generate, RuleLayer with width/overhang/spacing/enc)
- `LefSite` — Placement site (name, class: CORE|PAD|IO, symmetry: X|Y|R90|NONE, width, height)
- `LefPin` — Cell pin (name, direction: INPUT|OUTPUT|INOUT|FEEDTHRU, use: SIGNAL|POWER|GROUND|CLOCK|ANALOG, ports with rects, capacitance, max_fanout)
- `LefObs` — Obstruction layers with rects
- `LefMacro` — Cell definition (name, macro_class: CORE|CORE_TIEHIGH|...|PAD_*|BLOCK|ENDCAP_*|RING|COVER, width/height, origin, symmetry, site_ref, pins, obs, is_filler, properties)
- `LefSpacing` — Inter-layer spacing rule (layer1, layer2, min_spacing, stack)
- `LefLibrary` — Container (version, bus_bit_chars, divider_char, units, layers, vias, via_rules, sites, macros, spacings, properties)

**Public API**:
- `parse_file(filename) → LefLibrary`
- `parse_string(content) → LefLibrary`
- `find_macro(name) → const LefMacro*`
- `find_layer(name) → const LefLayer*`
- `find_via(name) → const LefVia*`
- `find_site(name) → const LefSite*`
- `num_routing_layers() → int`
- `num_cut_layers() → int`

---

### 2.6 DEF Parser (`core/def_parser.hpp`)

**File Path**: `core/def_parser.hpp`

**Key Class**: `DefParser`
- **Parse**: `parse_string(content, pd) → bool`, `parse_file(filename, pd) → bool`
- **Export**: `export_def(pd) → std::string`
- **Internal**: Tokenizer, token parser, cell/net/pin placement parsers

---

### 2.7 Design Database (`core/design_db.hpp`)

**File Path**: `core/design_db.hpp`

**Structures**:
- `DesignState` — Design stage snapshot
  - **Stage enum**: INIT | RTL_PARSED | SYNTHESIZED | PLACED | ROUTED | VERIFIED | SIGNED_OFF
  - **Members**: stage, design_name, netlist, physical, constraints, timing, power, technology, target_freq_mhz, version, log
- `DesignDbStats` — Statistical summary (designs, total_gates, total_nets, total_cells, message)

**Key Class**: `DesignDatabase`
- **Design management**: `create(name, tech)`, `get(name) → DesignState&`, `exists(name) → bool`, `remove(name)`
- **Workflow**: `set_stage(name, stage)`, `log(name, msg)`
- **Snapshots**: `snapshot(name) → int` — Version numbering
- **Reporting**: `stats() → DesignDbStats`, `list() → std::vector<std::string>`, `summary(name) → std::string`

---

### 2.8 Hierarchy Manager (`core/hierarchy.hpp`)

**File Path**: `core/hierarchy.hpp`

**Structures**:
- `DesignModule` — Module definition (name, netlist, physical, is_leaf, sub_modules, area_um2, gate_count)
- `HierarchyResult` — Analysis result (total_modules, leaf_modules, hierarchy_depth, total_area, total_gates, time_ms, message)

**Key Class**: `HierarchyManager`
- **Module management**: `add_module(name, nl)`, `add_module(name, nl, pd)`
- **Instantiation**: `instantiate(parent, child, instance_name)`
- **Analysis**: `analyze() → HierarchyResult`, `flatten(top_module) → Netlist`
- **Query**: `get_module(name) → DesignModule*`, `hierarchy_tree(top, indent) → std::string`

---

### 2.9 Die-to-Die / 3D-IC (`core/die_to_die.hpp`)

**File Path**: `core/die_to_die.hpp`

**TSV (Through-Silicon Via) Model**:
- `TsvParasitics` — Physics-based RLC (resistance Ω, inductance H, capacitance F, coupling_cap F, delay_ps, bandwidth_gbps)
  - **Formulas**: R = ρ·h/(π·r²), L = μ₀·h/(2π)·[ln(2h/r)-1], C = 2π·ε₀·εᵣ·h/ln(r_ox/r_tsv)
- `TsvTechParams` — TSV technology constants
  - **Parameters**: radius_um=2.5, height_um=50, oxide_thick_um=0.5, pitch_um=10, cu_resistivity=1.68e-8, oxide_er=3.9, si_er=11.7, mu0=1.2566e-6, eps0=8.854e-12, max_current_ma=20
  - **Computation**: `compute_parasitics() → TsvParasitics`, `compute_coupling(dist) → TsvParasitics`

**Micro-Bump Model**:
- `MicroBump` — Chiplet interconnect (id, net_id, die_top/bottom, x, y, contact_resistance=0.015Ω, inductance_ph=5, capacitance_ff=20, max_current_ma=150, is_power, is_spare)
- `MicroBumpTech` — Micro-bump technology (pitch_um=40, diameter_um=25, contact_r_ohm=0.015, inductance_ph=5, capacitance_ff=20, max_current_ma=150)

**Interposer Model** (2.5D):
- `InterposerTrace` — RDL trace (id, net_id, layer, x0/y0/x1/y1, width_um=2, r_per_um=0.025Ω/μm, c_per_um=0.15fF/μm)
  - **Methods**: `length_um() → double`, `resistance() → double`, `capacitance_ff() → double`
- `InterposerConfig` — Configuration (width_um=20000, height_um=20000, num_rdl_layers=4, trace_r_per_um, trace_c_per_um, trace_width_um, trace_pitch_um)

**3D Links**:
- `D2DLink` — Die-to-die communication link (UCIe-inspired) (id, from_die, to_die, num_lanes=16, lane_rate_gbps=32, latency_ns=2, power_mw_per_lane=0.5)
  - **Methods**: `bandwidth_gbps() → double`, `total_power_mw() → double`

**Power Network**:
- `PowerTSV` — Power delivery TSV (id, x, y, is_vdd, resistance=0.005Ω, current_ma)
- `PowerGrid3D` — 3D power grid model (grid_nx=16, grid_ny=16, num_dies=2, power_tsvs, ir_drop_map)
  - **Methods**: `max_ir_drop() → double`, `avg_ir_drop() → double`, `hotspot_count(threshold_mv=50) → int`

**Yield & Reliability**:
- `YieldModel` — Defect modeling (tsv_defect_rate=1e-5, bump_defect_rate=5e-6, die_defect_density=0.5, die_area_cm2=1)
  - **Methods**: `single_die_yield() → double` (Y = e^(-D·A)), `tsv_array_yield(num_tsvs, spares) → double`, `bump_array_yield()`, `package_yield()`

**Design Structures**:
- `DieInstance` — Die in stack/interposer (id, name, z_layer, x/y_offset, width/height_um, power_w, temperature_c)
- `TSV` — Signal TSV connection (id, net_id, from_die_id, to_die_id, x, y, is_signal, is_spare, parasitics)
- `ChipletReport` — Full 3D design report with TSV/bump/link statistics

**Key Class**: `PackageDesign`
- **Die management**: `add_die(name, z_layer, x, y) → int`, `set_die_size(id, w, h)`, `set_die_power(id, power)`
- **TSV management**: `add_tsv(net_id, from, to, x, y) → int`, `add_signal_tsv()`, `add_power_tsv()`, `add_spare_tsv()`
- **Micro-bumps**: `add_micro_bump(net_id, die_top, die_bottom, x, y) → int`, `add_power_bump()`
- **Interposer**: `add_interposer_trace(net_id, layer, x0, y0, x1, y1) → int`
- **Links**: `add_d2d_link(from_die, to_die, lanes, rate_gbps) → int`
- **Power**: `build_power_grid(nx, ny)`, `compute_ir_drop(die_id) → double`
- **Yield**: `estimate_yield(model) → double`, `compute_tsv_si(tsv_id) → TsvParasitics`, `worst_case_tsv_delay_ps() → double`, `total_d2d_bandwidth_gbps() → double`
- **Redundancy**: `assign_spare_tsv(failed_tsv_id) → int`, `count_spare_tsvs() → int`
- **Reporting**: `generate_report(tsv_tech, yield) → ChipletReport`

---

## 3. SYNTHESIS SUBSYSTEM

### 3.1 AIG Optimizer (`synth/aig_opt.hpp`)

**File Path**: `synth/aig_opt.hpp`

**Structures**:
- `OptStats` — Optimization results (initial_ands, final_ands, initial_depth, final_depth, passes, time_ms)
- `Cut` — Cut enumeration (leaves, root, size) with max_cut_size=4

**Key Class**: `AigOptimizer`
- **Main API**: `optimize(num_passes=3) → OptStats` — Runs all passes iteratively
- **Individual passes**:
  - `rewrite()` — Local rewriting with 4-input cuts (reference: Mishchenko DAC 2006)
  - `refactor()` — Collapse and re-derive sub-circuits
  - `balance()` — Tree-height reduction via associative balancing
  - `sweep()` — Constant propagation and dead node elimination
  - `sat_sweep()` — SAT-based redundancy removal
- **Analysis**: `compute_depth(aig) → uint32_t` — Longest path from input to output
- **Internal**: `enumerate_cuts(var, max_cut_size=4) → std::vector<Cut>`, `rebuild_from_truth_table(tt, leaves) → AigLit`

---

### 3.2 Technology Mapper (`synth/tech_mapper.hpp`)

**File Path**: `synth/tech_mapper.hpp`

**Structures**:
- `MapStats` — Mapping results (num_cells, total_area, depth, time_ms)
- `CellMatch` — Cell match to AIG pattern (cell: const LibertyCell*, inputs, output)

**Key Class**: `TechMapper`
- **Main API**: `map(optimize_area=true) → Netlist` — Map AIG to gate-level netlist
- **Algorithm**: Cut-enumeration and area/delay-optimal covering (reference: FlowMap, Cong & Ding IEEE TCAD 1994)
- **Reporting**: `stats() → const MapStats&`
- **Internal**: `match_node(var) → CellMatch`, `build_netlist(matches) → Netlist`, `insert_buffers(nl, max_fanout=8)`

---

### 3.3 Behavioral Synthesizer (`synth/behavioral_synth.hpp`)

**File Path**: `synth/behavioral_synth.hpp`

**Enums & Structures**:
- `AstNodeType` enum — 30+ node types (MODULE, PORT_DECL, ASSIGN, ALWAYS_POS_CLK, ALWAYS_COMB, IF_ELSE, NONBLOCK_ASSIGN, BIN_OP, UNARY_OP, TERNARY_OP, CONCAT, REPLICATE, CASE_STMT, FOR_LOOP, MEM_READ, MEM_WRITE)
- `AstNode` — AST node (type, value, children: shared_ptr<AstNode>, int_val)
- `BehavioralAST` — AST container (module_name, root, bus_ranges, net_map, memory_arrays, signed_signals)
- `BlockState` — Execution context (clock_net, is_combinational)

**Key Classes**:
- `BehavioralSynthesizer` — Synthesizes behavioral AST to gate-level netlist
  - **Main API**: `synthesize(ast, nl) → bool`
  - **Expression synthesis**: `synth_expr_single()`, `synth_expr_bus()` — Bus-aware multi-bit
  - **Arithmetic builders**:
    - Single-bit: `build_adder()`, `build_subtractor()`, `build_mux2()`, `build_comparator_lt()`, `build_comparator_gt()`
    - Multi-bit: `build_bus_adder()`, `build_bus_subtractor()`, `build_bus_multiplier()`, `build_bus_shift_left()`, `build_bus_shift_right()`, `build_barrel_shift_left()`, `build_barrel_shift_right()`, `build_arith_shift_right()`, `build_barrel_arith_shift_right()`
    - Comparison: `build_bus_lt()`, `build_bus_lt_signed()`, `build_bus_eq()`, `build_bus_mux()`
  - **Memory**: `synth_mem_read()`, `synth_mem_write()`
  - **Block handling**: `synth_always()`, `synth_always_comb()`, `synth_statement()`
  - **Helpers**: `get_bus_width()`, `infer_expr_width()`, `pad_to_width()`, `parse_number()`, `reduce()`, `bitwise_op()`

- `StructuralSynthesizer` — Synthesizes continuous assignments and combinational logic
  - **API**: `synth_expr()`, `synth_expr_bus()`

**Key Features**:
- **Bus-aware synthesis** with multi-bit signal tracking
- **Memory array synthesis** with word_width and depth
- **Signed arithmetic** via `signed_signals_` set (auto-propagates to comparisons)
- **Division/modulo** builder: `build_divider() → {quotient, remainder}`
- **Expression precedence** through nested synthesizers

---

### 3.4 Register Retiming (`synth/retiming.hpp`)

**File Path**: `synth/retiming.hpp`

**Structures**:
- `RetimingConfig` — Industrial configuration
  - **Preservation flags**: preserve_clock_gating, preserve_reset_paths, honor_multicycle_paths, hold_aware
  - **Resource constraints**: max_register_increase, area_budget
  - **Timing**: hold_margin, setup_margin, use_liberty_delays
  - **Optimization**: binary_search_steps=12, improvement_threshold=0.01
- `MulticyclePath` — Multi-cycle path exception (from_gate, to_gate, cycles)
- `RetimingResult` — Detailed optimization report
  - **Metrics**: critical_path_before/after, registers_moved, dffs_inserted/removed, improved
  - **Industrial metrics**: clock_gating_preserved, reset_paths_preserved, multicycle_paths_honored, register_count_before/after, area_before/after, fmax_before/after, fmax_improvement_pct, iterations

**Key Class**: `RetimingEngine`
- **Main API**: `optimize(nl) → bool`, `optimize_with_result(nl) → RetimingResult` (reference: Leiserson-Saxe algorithm)
- **Configuration**: `set_config(cfg)`, `config() → RetimingConfig&`
- **Library integration**: `set_liberty(lib) → void`
- **Constraints**: `add_dont_retime(gid)`, `clear_dont_retime()`, `add_multicycle_path(mcp)`, `add_clock_gating_cell(gid)`
- **Internal graph**: `RetimeNode` (gid, delay, r_val, frozen), `RetimeEdge` (u, v, weight, delay, multicycle_factor)
- **Algorithm phases**:
  1. `build_graph(nl)` — Construct constraint graph
  2. `detect_clock_gating(nl)` — Auto-detect ICG cells
  3. `detect_reset_paths(nl)` — Auto-detect reset-driven gates
  4. `apply_multicycle_relaxation()` — Apply MCP factor to edges
  5. `compute_critical_path() → double` — Bellman-Ford longest path (zero-weight)
  6. `compute_retiming_values(target_period) → bool` — Binary search for achievable period
  7. `apply_retiming(nl) → int` — Insert/remove DFFs per r-values
- **Industrial features**: Preserve ICG, reset paths, multi-cycle paths, hold time aware

---

### 3.5 Clock Gating (`synth/clock_gating.hpp`)

**File Path**: `synth/clock_gating.hpp`

**Structures**:
- `ClockGatingResult` — Gating report
  - **Stats**: total_ffs, gated_ffs, icg_cells_inserted, power_reduction_pct, time_ms
  - **IcgCell** — Inserted ICG (name, clk_in, enable, clk_out, gated_ffs)
- `FfGroup` — Group of FFs sharing same enable (clock, enable, ffs)

**Key Class**: `ClockGatingEngine`
- **Main API**: `insert() → ClockGatingResult` (reference: Benini & De Micheli, Dynamic Power Management)
- **Configuration**: `set_min_group(n)`, `set_activity(act)` — Activity factor default=0.1 (10%)
- **Algorithm**: Groups FFs by (clock, enable) pair and inserts ICG cells if group_size ≥ min_group_

---

### 3.6 ECO (Engineering Change Order) (`synth/eco.hpp`)

**File Path**: `synth/eco.hpp`

**Structures**:
- `EcoChange` — Change record
  - **Type enum**: ADD_GATE, REMOVE_GATE, REPLACE_GATE, ADD_NET, REMOVE_NET, RECONNECT, ADD_BUFFER
  - **Fields**: description, gate_id, net_id, new_type, name
- `EcoResult` — ECO execution report (changes_applied, gates_added/removed, buffers_inserted, nets_modified, success, message, changelog)

**Key Class**: `EcoEngine`
- **Atomic operations**:
  - `insert_buffer(net, name) → GateId` — Add buffer on net
  - `remove_gate(gid) → bool` — Remove gate
  - `replace_gate(gid, new_type) → GateId` — Replace with different type
  - `split_net(nid, name) → NetId` — Insert intermediate net
  - `reconnect(gid, input_idx, new_net) → bool` — Remap gate input
- **High-level ECO**:
  - `fix_timing(critical_nets) → EcoResult` — Insert buffers on critical paths
  - `fix_fanout(max_fanout) → EcoResult` — Fix high-fanout nets
- **Tracking**: `changes() → const std::vector<EcoChange>&`, `apply() → EcoResult`

---

### 3.7 FSM Extraction (`synth/fsm_extract.hpp`)

**File Path**: `synth/fsm_extract.hpp`

**Structures**:
- `StateMachine` — Extracted FSM (state_var_name, num_states, state_names, transitions: map<from_state, map<to_state, condition>>, encoded_values: map<state_name, int>)
- `FsmEncoding` enum — BINARY | ONE_HOT | GRAY_CODE

**Key Class**: `FsmExtractor`
- **Pass 1**: `extract_fsms(root) → std::vector<StateMachine>` — Identifies FSMs in behavioral AST
- **Pass 2**: `optimize_fsms(root, fsms, encoding)` — Re-encodes state variables to target encoding (default: ONE_HOT)
- **Internal**: `find_state_machines()`, `extract_transitions()`, `apply_encoding()`, `rewrite_ast()`

---

### 3.8 Multi-Vt Optimizer (`synth/multi_vt.hpp`)

**File Path**: `synth/multi_vt.hpp`

**Enums & Structures**:
- `VtType` enum — SVT (Standard) | LVT (Low) | HVT (High)
- `MultiVtConfig` — Technology parameters (lvt_speed_factor=0.7, hvt_speed_factor=1.4, lvt_leakage_factor=10, hvt_leakage_factor=0.1, timing_margin=0.1)
- `MultiVtResult` — Assignment report
  - **Stats**: total_cells, svt/lvt/hvt_cells, leakage_reduction_pct, timing_impact_pct, time_ms
  - **CellAssignment** — Per-cell Vt choice
- `MultiVtOptimizer` — Optimizer

**Key Class**: `MultiVtOptimizer`
- **Main API**: `optimize() → MultiVtResult` (reference: Sirichotiyakul et al., DAC 1999 — Simultaneous Vt Selection and Circuit Sizing)
- **Configuration**: `set_config(cfg)`
- **Algorithm**:
  - Compute logic depth via `compute_depth(gid) → int`
  - Assign LVT to critical path, HVT to non-critical paths
  - Respect timing_margin to preserve slack

---

### 3.9 Resource Sharing (`synth/resource_share.hpp`)

**File Path**: `synth/resource_share.hpp`

**Structures**:
- `ResourceOp` — Shareable operation (op_type: "+", "*", etc., node_ref, control_path: conditions to reach op)
- `ResourceSharer` — Area optimization via resource sharing

**Key Class**: `ResourceSharer`
- **Main API**: `optimize(root)` — Modifies AST to share mutually exclusive operations
- **Algorithm**: 
  - `extract_ops()` — Find operations with control paths
  - `are_mutually_exclusive(pathA, pathB) → bool` — Detect exclusive regions (e.g., different IF/ELSE branches)
  - `apply_sharing()` — Replace multiple ops with single shared instance + MUX select

---

## 4. SAT SUBSYSTEM

### 4.1 CDCL Solver (`sat/cdcl_solver.hpp`)

**File Path**: `sat/cdcl_solver.hpp`

**Enums & Types**:
- `LBool` enum (uint8_t) — UNDEF(0) | TRUE(1) | FALSE(2) — SAT solver state
- **Internal literal encoding**: var v → pos=2v, neg=2v+1

**Structures**:
- `Clause` — Clause representation (lits: vector of internal lit indices, learned: bool)
- `Stats` — Solver statistics (decisions, propagations, conflicts, restarts) — uint64_t counters

**Key Class**: `CdclSolver : public SatSolver`
- **Public API**:
  - `solve() → SatResult` — Main CDCL solver
  - `solve(assumptions) → SatResult` — Solve with literals forced true
  - `model_value(var) → bool` — Query model
  - `add_clause(clause)` — Dynamic clause addition
  - `new_var() → int` — Create variable
  - `num_vars() → int`
  - `stats() → const Stats&`
- **Algorithms**:
  1. **2-watched literal propagation** (`propagate() → int`) — Unit propagation returning conflict clause
  2. **First-UIP conflict analysis** (`analyze(confl, learnt, bt_level)`) — Derive learned clause and backtrack level
  3. **Non-chronological backjumping** (`backtrack(level)`)
  4. **VSIDS variable activity heuristic** (`bump_activity(var)`, `decay_activity()`, `pick_branch() → int`)
- **Internal state**:
  - `num_vars_` — Variable count
  - `clauses_` — Original + learned clauses
  - `assigns_` — Variable assignments (LBool)
  - `trail_` — Variable assignment history
  - `trail_lim_` — Decision level boundaries
  - `reason_` — Antecedent clause for each variable
  - `level_` — Decision level per variable
  - `watches_` — 2-watched-literal arrays
  - `activity_` — Variable activity scores
  - `var_inc_` — Activity increment (geometrically increased per conflict)
  - `qhead_` — Propagation queue head
  - `ok_` — Solver state (UNSAT if false)
- **Helper functions**:
  - `to_ilit(CnfLit l) → int` — External to internal literal
  - `neg(il) → int` — Negate internal literal
  - `ivar(il) → int` — Extract variable from internal literal
  - `ensure_var(v)` — Allocate variable
  - `decision_level() → int`
  - `val(ilit) → LBool`
  - `assign_lit(ilit, reason_cl)` — Assign literal with reason

---

## SUMMARY TABLE

| Subsystem | Module | Classes | Key Algorithms | Thresholds |
|-----------|--------|---------|-----------------|------------|
| **Frontend** | Verilog Parser | VerilogParser | Precedence expr, Structural hashing, Module elaboration | MAX_HIERARCHY_DEPTH=16 |
| | SDC Parser | SdcParser | TCL-like tokenization, Constraint parsing | — |
| | SVA Parser | SvaParser | LTL property parsing, Recursive descent | — |
| | UPF Parser | UpfParser, UpfChecker | Power domain hierarchy, DRC validation | IEEE 1801 |
| **Core** | Types | BitVector, Logic4 | 4-state IEEE logic, Truth tables | — |
| | Netlist | Netlist | Gate evaluation, Topological sort | — |
| | AIG | AigGraph, AigVec | Structural hashing, DAG-aware, AIGER encoding | strash dedup |
| | Liberty Parser | LibertyLibrary | NLDM 2D tables, Recursive descent | — |
| | LEF Parser | LefLibrary | LEF 5.8 parsing, Via/macro definitions | — |
| | DEF Parser | DefParser | DEF format parsing | — |
| | Design Database | DesignDatabase | Multi-design management, Snapshots | 7 design stages |
| | Hierarchy Manager | HierarchyManager | Module flattening, Hierarchy analysis | — |
| | Die-to-Die | PackageDesign | TSV/micro-bump physics, 3D power, Yield | TSV R/L/C formulas |
| **Synthesis** | AIG Optimizer | AigOptimizer | Rewrite, refactor, balance, sweep | max_cut_size=4 |
| | Tech Mapper | TechMapper | FlowMap algorithm, Cut enumeration | — |
| | Behavioral Synth | BehavioralSynthesizer | Expr synthesis, Arithmetic builders, Memory arrays | — |
| | Register Retiming | RetimingEngine | Leiserson-Saxe algorithm, Bellman-Ford | binary_search_steps=12 |
| | Clock Gating | ClockGatingEngine | ICG insertion, Enable factoring | activity=0.1 |
| | ECO Engine | EcoEngine | Atomic ECO ops, Buffer insertion, Timing fix | — |
| | FSM Extractor | FsmExtractor | State machine extraction, Encoding (BINARY/ONE_HOT/GRAY) | — |
| | Multi-Vt | MultiVtOptimizer | Vt assignment by logic depth, Leakage optimization | timing_margin=0.1 |
| | Resource Sharing | ResourceSharer | Mutual exclusivity analysis, Operation sharing | — |
| **SAT** | CDCL Solver | CdclSolver | 2-watched literals, First-UIP analysis, VSIDS, Non-chron backjump | — |

---

## KEY TECHNICAL REFERENCES

1. **AIG**: Mishchenko et al., "DAG-Aware AIG Rewriting", DAC 2006
2. **Technology Mapping**: Cong & Ding, "FlowMap", IEEE TCAD 1994
3. **Retiming**: Leiserson & Saxe, "Retiming Synchronous Circuitry", Algorithmica 1991
4. **Register Retiming with Constraints**: Pan, "Retiming Under Setup and Hold Constraints", DAC 1999
5. **Clock Gating**: Benini & De Micheli, "Dynamic Power Management", Springer 1998
6. **Multi-Vt Optimization**: Sirichotiyakul et al., "Simultaneous Threshold Voltage Selection and Circuit Sizing", DAC 1999
7. **TSV/3D-IC**: Katti et al., "Electrical Modeling of TSVs", IEEE T-CPMT 2010; Pak et al., "TSV Parasitic Modeling", ECTC 2011
8. **Verilog/SystemVerilog**: IEEE 1364 (Verilog 2005), IEEE 1800 (SystemVerilog 2017)
9. **Power Intent (UPF)**: IEEE 1801 (UPF 2.1)
10. **Liberty/LEF/DEF**: Synopsys Liberty User Guide, Cadence LEF/DEF 5.8 Language Reference
11. **CDCL SAT Solving**: Marques-Silva & Sakallah, "GRASP: A Search Algorithm for Propositional Satisfiability"; Eén & Sörensson, "An Extensible SAT-solver"

