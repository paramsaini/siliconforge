# 🔍 COMPLETE DEEP ANALYSIS: DEF Parser & Hierarchy System

**Project**: SiliconForge  
**Analysis Date**: 2024  
**Files Analyzed**: 7 core files  

---

## 1️⃣ DEF PARSER: `src/core/def_parser.hpp`

### Class Definition
```cpp
class DefParser {
    // Public API
    bool parse_string(const std::string& content, PhysicalDesign& pd);
    bool parse_file(const std::string& filename, PhysicalDesign& pd);
    static std::string export_def(const PhysicalDesign& pd);
    
    // Private helpers
    std::vector<std::string> tokenize(const std::string& content);
    bool parse_tokens(const std::vector<std::string>& tokens, PhysicalDesign& pd);
};
```

### What DefParser Stores
- **NOTHING**: Stateless parser
- **Takes input**: std::string (DEF content)
- **Outputs to**: PhysicalDesign& (by reference)
- **Returns**: bool (success/failure)

---

## 2️⃣ DEF PARSER: `src/core/def_parser.cpp` — FULL PARSING LOGIC

### A. TOKENIZATION (Lines 10-34)
```cpp
std::vector<std::string> DefParser::tokenize(const std::string& content) {
    std::vector<std::string> tokens;
    std::istringstream ss(content);
    std::string tok;
    while (ss >> tok) {
        // SKIP COMMENTS: "#" starts comment to EOL
        if (tok.size() >= 1 && tok[0] == '#') {
            std::string line;
            std::getline(ss, line);  // ← discard rest of line
            continue;
        }
        // SPLIT DELIMITERS: ; ( ) become separate tokens
        std::string cur;
        for (char c : tok) {
            if (c == ';' || c == '(' || c == ')') {
                if (!cur.empty()) { tokens.push_back(cur); cur.clear(); }
                tokens.push_back(std::string(1, c));
            } else {
                cur += c;
            }
        }
        if (!cur.empty()) tokens.push_back(cur);
    }
    return tokens;
}
```

**What it does**: Converts DEF text → token stream
- Whitespace-aware splitting
- **Strips comments** (lines with #)
- **Breaks on**: semicolons and parentheses

### B. DIEAREA SECTION (Lines 39-52)

**Expected format**:
```
DIEAREA ( x0 y0 ) ( x1 y1 ) ;
```

**Code**:
```cpp
if (t[pos] == "DIEAREA") {
    pos++;
    // ( x0 y0 ) ( x1 y1 ) ;
    if (pos < t.size() && t[pos] == "(") pos++;
    double x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    if (pos < t.size()) { x0 = std::stod(t[pos]) / 1000.0; pos++; }  // ← DBU→um
    if (pos < t.size()) { y0 = std::stod(t[pos]) / 1000.0; pos++; }
    if (pos < t.size() && t[pos] == ")") pos++;
    if (pos < t.size() && t[pos] == "(") pos++;
    if (pos < t.size()) { x1 = std::stod(t[pos]) / 1000.0; pos++; }  // ← DBU→um
    if (pos < t.size()) { y1 = std::stod(t[pos]) / 1000.0; pos++; }
    if (pos < t.size() && t[pos] == ")") pos++;
    if (pos < t.size() && t[pos] == ";") pos++;
    pd.die_area = Rect(x0, y0, x1, y1);  // ← Store in physical design
}
```

**Key operation**: 
- Divides all coordinates by **1000** (converts from database units to microns)
- Stores in `PhysicalDesign::die_area`

### C. COMPONENTS SECTION (Lines 54-86)

**Expected format**:
```
COMPONENTS count ;
  - name type PLACED ( x y ) ... ;
  - name type FIXED ( x y ) ... ;
END COMPONENTS
```

**Code**:
```cpp
else if (t[pos] == "COMPONENTS") {
    pos++;           // skip "COMPONENTS"
    if (pos < t.size()) pos++;  // skip count
    if (pos < t.size() && t[pos] == ";") pos++;

    while (pos < t.size() && t[pos] != "END") {
        if (t[pos] == "-") {  // ← Each cell entry starts with "-"
            pos++;
            std::string name, type;
            if (pos < t.size()) { name = t[pos]; pos++; }      // Cell name
            if (pos < t.size()) { type = t[pos]; pos++; }      // Cell type

            double px = 0, py = 0;
            while (pos < t.size() && t[pos] != ";") {
                if (t[pos] == "PLACED" || t[pos] == "FIXED") {  // ← Placement keyword
                    pos++;
                    if (pos < t.size() && t[pos] == "(") pos++;
                    if (pos < t.size()) { px = std::stod(t[pos]) / 1000.0; pos++; }  // ← DBU→um
                    if (pos < t.size()) { py = std::stod(t[pos]) / 1000.0; pos++; }
                    if (pos < t.size() && t[pos] == ")") pos++;
                }
                else pos++;  // skip unknown keywords
            }
            if (pos < t.size() && t[pos] == ";") pos++;

            // ← CRITICAL: Hardcoded size
            int cid = pd.add_cell(name, type, 3.0, pd.row_height);
            pd.cells[cid].position = {px, py};
            pd.cells[cid].placed = true;
        }
        else pos++;
    }
    if (pos < t.size() && t[pos] == "END") { pos++; pos++; }  // END COMPONENTS
}
```

**Key operations**:
- Searches for `-` to identify cell entries
- Extracts: **name**, **type**, **position**
- Creates CellInstance with:
  - Width = **hardcoded 3.0 microns**
  - Height = **hardcoded pd.row_height** (default 10.0)
- Sets `placed = true`

### D. NETS SECTION (Lines 87-124)

**Expected format**:
```
NETS count ;
  - net_name ( cell_name pin ) ( cell_name pin ) ... ;
END NETS
```

**Code**:
```cpp
else if (t[pos] == "NETS") {
    pos++;  // skip "NETS"
    if (pos < t.size()) pos++;  // skip count
    if (pos < t.size() && t[pos] == ";") pos++;

    while (pos < t.size() && t[pos] != "END") {
        if (t[pos] == "-") {  // ← Each net entry starts with "-"
            pos++;
            std::string net_name;
            if (pos < t.size()) { net_name = t[pos]; pos++; }  // Net name

            std::vector<int> cell_ids;
            while (pos < t.size() && t[pos] != ";") {
                if (t[pos] == "(") {  // ← Connection tuple
                    pos++;
                    if (pos < t.size()) {
                        std::string cname = t[pos]; pos++;  // Cell name
                        // ← LINEAR SEARCH through all cells by name
                        for (size_t ci = 0; ci < pd.cells.size(); ++ci) {
                            if (pd.cells[ci].name == cname) {
                                cell_ids.push_back(ci);
                                break;
                            }
                        }
                    }
                    while (pos < t.size() && t[pos] != ")") pos++;  // skip to )
                    if (pos < t.size()) pos++;  // skip )
                }
                else pos++;  // skip unknown tokens
            }
            if (pos < t.size() && t[pos] == ";") pos++;
            
            // ← Only add net if at least 2 endpoints
            if (cell_ids.size() >= 2)
                pd.add_net(net_name, cell_ids);
        }
        else pos++;
    }
    if (pos < t.size() && t[pos] == "END") { pos++; pos++; }  // END NETS
}
```

**Key operations**:
- Finds `-` to identify net entries
- **Searches for `(` ... `)` tuples** containing cell names
- **Linear search**: finds cell by name in `pd.cells`
- Only adds net if **≥ 2 endpoints**
- Calls `pd.add_net(net_name, cell_ids)`

### E. DEFAULT CASE (Line 125)
```cpp
else pos++;  // ← Unknown tokens silently skipped
```

### Sections SKIPPED / NOT PARSED
| Section | Status | Reason |
|---------|--------|--------|
| LEF | ✗ Not parsed | Cell definitions external |
| BLOCKAGE | ✗ Skipped | No obstructions handled |
| REGION | ✗ Skipped | No region constraints |
| PIN | ✗ Skipped | Pin definitions ignored |
| VIA | ✗ Skipped | Vias not extracted |
| TRACK | ✗ Skipped | Routing tracks ignored |
| GCELLGRID | ✗ Skipped | No routing grid |
| ORIENTATION | Parsed but unused | Read in COMPONENTS, not applied |
| Routing | ✗ Stub | WireSegment vector empty |

### Code Flow Summary

```
parse_file(filename, PhysicalDesign&)
    ↓
open file → read all bytes
    ↓
parse_string(content, PhysicalDesign&)
    ↓
tokenize(content) → vector<string> tokens
    ↓
parse_tokens(tokens, PhysicalDesign&)
    ├─ pos = 0
    ├─ while pos < tokens.size()
    │   ├─ DIEAREA → die_area = Rect(...)
    │   ├─ COMPONENTS → cells.push_back(CellInstance)
    │   ├─ NETS → nets.push_back(PhysNet)
    │   └─ else → pos++ (skip unknown)
    └─ return true
```

---

## 3️⃣ HIERARCHY SYSTEM: `src/core/hierarchy.hpp`

### DesignModule Structure (Lines 16-24)
```cpp
struct DesignModule {
    std::string name;                      // "top", "cpu", "memory", etc.
    Netlist netlist;                       // ← LOGICAL: gate-level circuit
    PhysicalDesign physical;               // ← PHYSICAL: placement & routing
    bool is_leaf = true;                   // false if has sub-modules
    std::vector<std::string> sub_modules;  // Names of instantiated children
    double area_um2 = 0;                   // physical.die_area.area()
    int gate_count = 0;                    // netlist.num_gates()
};
```

**Key insight**: Each module is a PAIR of (netlist, physical design).

### HierarchyResult Structure (Lines 26-34)
```cpp
struct HierarchyResult {
    int total_modules = 0;          // Count across all modules
    int leaf_modules = 0;           // Modules with no sub-modules
    int hierarchy_depth = 0;        // Max depth from root to leaf
    double total_area = 0;          // Sum of all module areas
    int total_gates = 0;            // Sum of all gates
    double time_ms = 0;             // Analysis time (high_resolution_clock)
    std::string message;            // "N modules (L leaf), depth=D, G gates"
};
```

### HierarchyManager Class (Lines 36-62)
```cpp
class HierarchyManager {
    // Storage: map of module_name → DesignModule
    std::unordered_map<std::string, DesignModule> modules_;

public:
    // Add module (logical only)
    void add_module(const std::string& name, const Netlist& nl);
    
    // Add module (logical + physical)
    void add_module(const std::string& name, const Netlist& nl, 
                   const PhysicalDesign& pd);

    // Create parent→child instantiation
    void instantiate(const std::string& parent, const std::string& child,
                    const std::string& instance_name = "");

    // Analyze hierarchy tree
    HierarchyResult analyze();

    // Flatten all modules into single netlist
    Netlist flatten(const std::string& top_module);

    // Get module by name
    DesignModule* get_module(const std::string& name);
    const DesignModule* get_module(const std::string& name) const;

    // Print hierarchy tree with statistics
    std::string hierarchy_tree(const std::string& top, int indent = 0) const;

private:
    // Recursively compute depth
    int compute_depth(const std::string& mod, int current = 0) const;
};
```

---

## 4️⃣ HIERARCHY IMPLEMENTATION: `src/core/hierarchy.cpp`

### A. add_module() — Logical Only (Lines 10-17)
```cpp
void HierarchyManager::add_module(const std::string& name, const Netlist& nl) {
    DesignModule mod;
    mod.name = name;
    mod.netlist = nl;                        // ← Copy netlist
    mod.gate_count = (int)nl.num_gates();    // ← Extract gate count
    mod.is_leaf = true;
    modules_[name] = std::move(mod);
}
```

**Usage**: When you only have gate-level logic, no placement.

### B. add_module() — Logical + Physical (Lines 19-29)
```cpp
void HierarchyManager::add_module(const std::string& name, const Netlist& nl,
                                    const PhysicalDesign& pd) {
    DesignModule mod;
    mod.name = name;
    mod.netlist = nl;                              // ← Copy netlist
    mod.physical = pd;                             // ← Copy physical design
    mod.gate_count = (int)nl.num_gates();
    mod.area_um2 = pd.die_area.area();             // ← Extract die area
    mod.is_leaf = true;
    modules_[name] = std::move(mod);
}
```

**Usage**: Full hierarchical design with both logic and layout.

### C. instantiate() (Lines 31-37)
```cpp
void HierarchyManager::instantiate(const std::string& parent, 
                                    const std::string& child,
                                    const std::string& instance_name) {
    auto pit = modules_.find(parent);
    if (pit == modules_.end()) return;  // ← Parent must exist
    pit->second.sub_modules.push_back(child);  // ← Add child to parent's list
    pit->second.is_leaf = false;  // ← Mark parent as non-leaf
}
```

**Note**: `instance_name` parameter is **UNUSED** — only module references tracked.

### D. analyze() (Lines 59-88)

```cpp
HierarchyResult HierarchyManager::analyze() {
    auto t0 = std::chrono::high_resolution_clock::now();
    HierarchyResult r;
    r.total_modules = (int)modules_.size();

    // ← 1. COUNT LEAF MODULES & SUM GATES/AREA
    for (auto& [name, mod] : modules_) {
        if (mod.is_leaf) r.leaf_modules++;
        r.total_gates += mod.gate_count;
        r.total_area += mod.area_um2;
    }

    // ← 2. FIND ROOT MODULES (not instantiated by anyone)
    std::unordered_set<std::string> children;
    for (auto& [name, mod] : modules_)
        for (auto& sub : mod.sub_modules) 
            children.insert(sub);

    // ← 3. COMPUTE HIERARCHY DEPTH
    for (auto& [name, mod] : modules_) {
        if (children.find(name) == children.end()) {  // ← Is this a root?
            r.hierarchy_depth = compute_depth(name);   // ← Recurse from root
        }
    }

    auto t1 = std::chrono::high_resolution_clock::now();
    r.time_ms = std::chrono::duration<double, std::milli>(t1 - t0).count();
    r.message = std::to_string(r.total_modules) + " modules (" +
                std::to_string(r.leaf_modules) + " leaf), depth=" +
                std::to_string(r.hierarchy_depth) + ", " +
                std::to_string(r.total_gates) + " gates";
    return r;
}
```

**Algorithm**:
1. **Count leaf modules**: modules with empty sub_modules list
2. **Sum gates and area**: aggregate across all modules
3. **Identify roots**: modules not referenced as sub-modules
4. **Compute depth**: max depth from any root

### E. flatten() (Lines 90-124)

```cpp
Netlist HierarchyManager::flatten(const std::string& top_module) {
    Netlist flat;  // ← Output netlist
    
    std::function<void(const std::string&, const std::string&)> merge;
    merge = [&](const std::string& mod_name, const std::string& prefix) {
        auto* mod = get_module(mod_name);
        if (!mod) return;

        std::unordered_map<NetId, NetId> net_remap;
        
        // ← STEP 1: Copy nets with prefix
        for (size_t i = 0; i < mod->netlist.num_nets(); ++i) {
            auto& net = mod->netlist.net(i);
            NetId new_nid = flat.add_net(prefix + net.name);  // ← Hierarchical name
            net_remap[i] = new_nid;
        }
        
        // ← STEP 2: Copy gates (skip I/O), remap inputs/outputs
        for (size_t g = 0; g < mod->netlist.num_gates(); ++g) {
            auto& gate = mod->netlist.gate(g);
            
            // ← Skip INPUT and OUTPUT gates (these are interface points)
            if (gate.type == GateType::INPUT || gate.type == GateType::OUTPUT) 
                continue;
            
            // ← Remap input nets to flattened versions
            std::vector<NetId> new_inputs;
            for (auto inp : gate.inputs) {
                if (net_remap.count(inp)) 
                    new_inputs.push_back(net_remap[inp]);
            }
            
            // ← Remap output net
            NetId new_out = net_remap.count(gate.output) ? 
                           net_remap[gate.output] : -1;
            
            if (new_out >= 0)
                flat.add_gate(gate.type, new_inputs, new_out, 
                             prefix + gate.name);
        }

        // ← STEP 3: Recursively merge sub-modules
        for (auto& sub : mod->sub_modules)
            merge(sub, prefix + sub + "/");  // ← Deeper prefix
    };

    merge(top_module, "");  // ← Start from top with empty prefix
    return flat;
}
```

**Process**:
1. **Recursively traverse** hierarchy starting from top_module
2. **For each module**: add its nets/gates to flat netlist with hierarchical prefix
3. **Net remapping**: old NetId → new NetId (because new netlist has different IDs)
4. **Skip I/O gates**: only copy internal logic
5. **Final result**: Single netlist like `top/sub1/gate0`, `top/sub2/gate1`, etc.

### F. hierarchy_tree() (Lines 126-141)

```cpp
std::string HierarchyManager::hierarchy_tree(const std::string& top, int indent) const {
    std::ostringstream ss;
    auto* mod = get_module(top);
    if (!mod) return "";

    // ← Print indentation
    for (int i = 0; i < indent; ++i) ss << "  ";
    
    // ← Print module with statistics
    ss << (mod->is_leaf ? "├─ " : "├─ ") << top;
    ss << " (" << mod->gate_count << " gates";
    if (mod->area_um2 > 0) ss << ", " << (int)mod->area_um2 << "um²";
    ss << ")\n";

    // ← Recursively print sub-modules
    for (auto& sub : mod->sub_modules)
        ss << hierarchy_tree(sub, indent + 1);

    return ss.str();
}
```

**Output example**:
```
├─ top (100 gates, 5000um²)
  ├─ cpu (50 gates, 2500um²)
  ├─ memory (50 gates, 2500um²)
```

---

## 5️⃣ NETLIST: `src/core/netlist.hpp`

### GateType Enum (Lines 14-19)
```cpp
enum class GateType {
    INPUT, OUTPUT,                          // I/O ports
    BUF, NOT,                               // Unary
    AND, OR, NAND, NOR, XOR, XNOR,         // Logic gates
    MUX,                                    // Mux (3 inputs: sel, a, b)
    DFF, DLATCH,                            // Sequential
    TRI,                                    // Tri-state (not used)
    CONST0, CONST1,                         // Constants
    BUFIF0, BUFIF1, NOTIF0, NOTIF1         // Conditional buffers
};
```

### Type Aliases (Lines 47-48)
```cpp
using NetId = int32_t;    // Index into Netlist::nets_
using GateId = int32_t;   // Index into Netlist::gates_
```

### Net Structure (Lines 50-57)
```cpp
struct Net {
    NetId id;                          // Self-index in netlist
    std::string name;                  // Signal name (e.g., "clk", "reset")
    Logic4 value = Logic4::X;          // Current value (0/1/X/Z)
    Logic4 next_value = Logic4::X;     // Next value (for event-driven sim)
    std::vector<GateId> fanout;        // Gates reading this net
    GateId driver = -1;                // Gate driving this net (-1 if undriven)
};
```

**Connectivity graph**: Each net points to its driver gate AND fanout gates.

### Gate Structure (Lines 59-69)
```cpp
struct Gate {
    GateId id;                         // Self-index in netlist
    GateType type;                     // Gate type
    std::string name;                  // Instance name
    std::vector<NetId> inputs;         // Input nets (generic)
    NetId output = -1;                 // Output net
    // For DFF/DLATCH:
    NetId clk = -1;                    // Clock input
    NetId reset = -1;                  // Async reset
    Logic4 init_val = Logic4::ZERO;    // Initial state
};
```

**Design**:
- Generic `inputs` vector (fan-in can vary)
- Single `output` net
- **DFF fields**: clk, reset (separate from inputs)

### Netlist Class (Lines 71-109)
```cpp
class Netlist {
public:
    // Creation
    NetId add_net(const std::string& name = "");
    GateId add_gate(GateType type, const std::vector<NetId>& inputs, 
                    NetId output, const std::string& name = "");
    GateId add_dff(NetId d, NetId clk, NetId q, NetId reset = -1, 
                   const std::string& name = "");

    // Marking I/O
    void mark_input(NetId id);   // ← Add to pis_
    void mark_output(NetId id);  // ← Add to pos_

    // Accessors
    Net& net(NetId id)              { return nets_[id]; }
    const Net& net(NetId id) const  { return nets_[id]; }
    Gate& gate(GateId id)           { return gates_[id]; }
    const Gate& gate(GateId id) const { return gates_[id]; }

    size_t num_nets() const   { return nets_.size(); }
    size_t num_gates() const  { return gates_.size(); }
    
    const std::vector<NetId>& primary_inputs() const  { return pis_; }
    const std::vector<NetId>& primary_outputs() const { return pos_; }
    const std::vector<GateId>& flip_flops() const     { return dffs_; }
    
    const std::vector<Net>& nets() const   { return nets_; }
    const std::vector<Gate>& gates() const { return gates_; }

    // Analysis
    std::vector<GateId> topo_order() const;      // Kahn's algorithm
    static Logic4 eval_gate(GateType type, const std::vector<Logic4>& inputs);

    // Maintenance
    void print_stats() const;
    void clear();

private:
    std::vector<Net> nets_;         // All nets
    std::vector<Gate> gates_;       // All gates
    std::vector<NetId> pis_, pos_;  // Primary inputs/outputs
    std::vector<GateId> dffs_;      // DFF cells
};
```

**Key property**: Netlist is **ALWAYS FLAT** — no hierarchy/modules.

---

## 6️⃣ NETLIST IMPLEMENTATION: `src/core/netlist.cpp`

### A. add_net() (Lines 10-13)
```cpp
NetId Netlist::add_net(const std::string& name) {
    NetId id = (NetId)nets_.size();
    nets_.push_back({id, name.empty() ? ("n" + std::to_string(id)) : name});
    return id;
}
```

**Auto-naming**: If name is empty, generates `n0`, `n1`, etc.

### B. add_gate() (Lines 16-30)
```cpp
GateId Netlist::add_gate(GateType type, const std::vector<NetId>& inputs,
                          NetId output, const std::string& name) {
    GateId id = (GateId)gates_.size();
    Gate g{id, type, name.empty() ? /* auto-name */ : name, inputs, output};
    gates_.push_back(g);

    // ← BIDIRECTIONAL: Connect inputs
    for (auto ni : inputs)
        nets_[ni].fanout.push_back(id);  // ← This gate reads net ni
    
    // ← BIDIRECTIONAL: Connect output
    if (output >= 0)
        nets_[output].driver = id;  // ← This gate drives net output

    return id;
}
```

**Maintains dual pointers**: 
- `nets_[i].fanout` ← all gates reading net i
- `gates_[i].inputs` ← all nets read by gate i

### C. add_dff() (Lines 32-46)
```cpp
GateId Netlist::add_dff(NetId d, NetId clk, NetId q, NetId reset,
                         const std::string& name) {
    GateId id = (GateId)gates_.size();
    Gate g{id, GateType::DFF, name, {d}, q};  // ← Single input d
    g.clk = clk;
    g.reset = reset;
    gates_.push_back(g);
    
    // ← Connect fanout (3 inputs: d, clk, reset)
    nets_[d].fanout.push_back(id);
    nets_[clk].fanout.push_back(id);
    if (reset >= 0) nets_[reset].fanout.push_back(id);
    
    // ← Connect driver
    if (q >= 0) nets_[q].driver = id;
    
    // ← Track as sequential
    dffs_.push_back(id);
    return id;
}
```

**DFF connectivity**: 
- `inputs = {d}` (only data input)
- `clk`, `reset` stored separately

### D. eval_gate() (Lines 51-75)

Truth table evaluator using 4-state logic:

```cpp
Logic4 Netlist::eval_gate(GateType type, const std::vector<Logic4>& in) {
    switch (type) {
        case GateType::BUF:
            return in[0];
        
        case GateType::NOT:
            return logic_not(in[0]);
        
        case GateType::AND:
            { Logic4 r = Logic4::ONE; 
              for (auto v : in) r = logic_and(r, v); 
              return r; }
        
        case GateType::OR:
            { Logic4 r = Logic4::ZERO; 
              for (auto v : in) r = logic_or(r, v); 
              return r; }
        
        case GateType::NAND:
            { Logic4 r = Logic4::ONE; 
              for (auto v : in) r = logic_and(r, v); 
              return logic_not(r); }
        
        case GateType::NOR:
            { Logic4 r = Logic4::ZERO; 
              for (auto v : in) r = logic_or(r, v); 
              return logic_not(r); }
        
        case GateType::XOR:
            return logic_xor(in[0], in[1]);
        
        case GateType::XNOR:
            return logic_not(logic_xor(in[0], in[1]));
        
        case GateType::MUX:
            // in[0] = select, in[1] = if_1, in[2] = if_0
            return (in[0] == Logic4::ONE) ? in[1] : 
                   (in[0] == Logic4::ZERO) ? in[2] : Logic4::X;
        
        case GateType::CONST0:
            return Logic4::ZERO;
        
        case GateType::CONST1:
            return Logic4::ONE;
        
        // BUFIF1: output = enable ? data : Z
        case GateType::BUFIF1:
            return (in.size() >= 2 && in[1] == Logic4::ONE) ? in[0] : Logic4::X;
        
        // BUFIF0: output = !enable ? data : Z
        case GateType::BUFIF0:
            return (in.size() >= 2 && in[1] == Logic4::ZERO) ? in[0] : Logic4::X;
        
        // NOTIF1: output = enable ? ~data : Z
        case GateType::NOTIF1:
            return (in.size() >= 2 && in[1] == Logic4::ONE) ? 
                   logic_not(in[0]) : Logic4::X;
        
        // NOTIF0: output = !enable ? ~data : Z
        case GateType::NOTIF0:
            return (in.size() >= 2 && in[1] == Logic4::ZERO) ? 
                   logic_not(in[0]) : Logic4::X;
        
        default:
            return Logic4::X;
    }
}
```

**4-state logic rules** (from IEEE 1364):
- AND: 0 dominates, then X, then Z; 1&1=1
- OR: 1 dominates, then X, then Z; 0|0=0
- XOR: X if any input is X/Z

### E. topo_order() — Kahn's Algorithm (Lines 77-110)

```cpp
std::vector<GateId> Netlist::topo_order() const {
    std::vector<int> in_degree(gates_.size(), 0);
    
    // ← STEP 1: Compute in-degrees for combinational gates
    for (auto& g : gates_) {
        // ← Skip DFFs and INPUTs (they are sources)
        if (g.type == GateType::DFF || g.type == GateType::INPUT) continue;
        
        for (auto ni : g.inputs) {
            if (nets_[ni].driver >= 0) {
                auto& drv = gates_[nets_[ni].driver];
                // ← Only count non-source gates
                if (drv.type != GateType::DFF && drv.type != GateType::INPUT)
                    in_degree[g.id]++;
            }
        }
    }

    // ← STEP 2: Enqueue gates with zero in-degree
    std::queue<GateId> q;
    for (auto& g : gates_) {
        if (g.type == GateType::DFF || g.type == GateType::INPUT) continue;
        if (in_degree[g.id] == 0) q.push(g.id);
    }

    // ← STEP 3: Process queue, decrement fanout gates
    std::vector<GateId> order;
    while (!q.empty()) {
        GateId gid = q.front(); q.pop();
        order.push_back(gid);
        
        auto& g = gates_[gid];
        if (g.output >= 0) {
            // ← For each gate reading this output
            for (auto fo_gid : nets_[g.output].fanout) {
                if (gates_[fo_gid].type == GateType::DFF) continue;  // ← Stop at DFFs
                if (--in_degree[fo_gid] == 0) q.push(fo_gid);
            }
        }
    }
    return order;
}
```

**Result**: Combinational gates in evaluation order (DFFs cut feedback loops).

---

## 7️⃣ PHYSICAL DESIGN: `src/pnr/physical.hpp` + `.cpp`

### Point & Rect Structs (Lines 14-36)

```cpp
struct Point {
    double x = 0, y = 0;
    Point() = default;
    Point(double x, double y) : x(x), y(y) {}
    
    Point operator+(const Point& o) const { return {x+o.x, y+o.y}; }
    Point operator-(const Point& o) const { return {x-o.x, y-o.y}; }
    Point operator*(double s) const { return {x*s, y*s}; }
    double dist(const Point& o) const { return std::abs(x-o.x) + std::abs(y-o.y); }  // Manhattan
};

struct Rect {
    double x0 = 0, y0 = 0, x1 = 0, y1 = 0;
    
    double width() const  { return x1 - x0; }
    double height() const { return y1 - y0; }
    double area() const   { return width() * height(); }
    Point center() const  { return {(x0+x1)/2, (y0+y1)/2}; }
    
    bool contains(const Point& p) const 
        { return p.x >= x0 && p.x <= x1 && p.y >= y0 && p.y <= y1; }
    
    bool overlaps(const Rect& o) const
        { return !(x1 <= o.x0 || o.x1 <= x0 || y1 <= o.y0 || o.y1 <= y0); }
};
```

### CellInstance Structure (Lines 39-49)

```cpp
struct CellInstance {
    int id = -1;              // Index in PhysicalDesign::cells[]
    std::string name;         // Instance name
    std::string cell_type;    // Cell definition reference
    double width = 1.0;       // Size (um)
    double height = 1.0;
    Point position;           // Bottom-left corner
    bool placed = false;      // Placement status flag
    int orientation = 0;      // 0=N, 1=S, 2=W, 3=E, 4=FN, etc.
    bool is_macro = false;    // Hard macro flag
    double halo = 5.0;        // Keepout zone (um)
};
```

**Key**: NO REFERENCE TO NETLIST. CellInstance is purely physical.

### PhysNet Structure (Lines 52-57)

```cpp
struct PhysNet {
    int id = -1;
    std::string name;
    std::vector<int> cell_ids;        // Which cells this net touches
    std::vector<Point> pin_offsets;   // Pin position within each cell
};
```

**Note**: Connects cells, not gates. Offset = pin location relative to cell origin.

### RoutingLayer, WireSegment, Via (Lines 60-81)

```cpp
struct RoutingLayer {
    int id;
    std::string name;
    bool horizontal;  // Preferred direction
    double pitch;     // Distance between tracks
    double width;     // Wire width
    double spacing;   // Minimum spacing
};

struct WireSegment {
    int layer;
    Point start, end;
    double width;
    int net_id = -1;  // Owning net (-1 = unknown)
};

struct Via {
    Point position;
    int lower_layer, upper_layer;
};
```

### PhysicalDesign Class (Lines 84-105)

```cpp
class PhysicalDesign {
public:
    Rect die_area;
    double row_height = 10.0;
    double site_width = 1.0;

    std::vector<CellInstance> cells;
    std::vector<PhysNet> nets;
    std::vector<RoutingLayer> layers;
    std::vector<WireSegment> wires;   // ← STUB: usually empty
    std::vector<Via> vias;            // ← STUB: usually empty

    int add_cell(const std::string& name, const std::string& type, 
                 double w, double h);
    int add_net(const std::string& name, const std::vector<int>& cell_ids);

    double total_wirelength() const;    // HPWL
    double utilization() const;         // cell area / die area
    bool has_overlaps() const;
    void print_stats() const;
};
```

### Implementation: add_cell() (Lines 8-12)

```cpp
int PhysicalDesign::add_cell(const std::string& name, const std::string& type,
                              double w, double h) {
    int id = (int)cells.size();
    cells.push_back({id, name, type, w, h, {0,0}, false});
    //                    ↑   ↑     ↑    ↑  ↑  ↑    ↑
    //                   id  name  type w  h pos placed
    return id;
}
```

### Implementation: add_net() (Lines 15-24)

```cpp
int PhysicalDesign::add_net(const std::string& name, 
                            const std::vector<int>& cell_ids) {
    int id = (int)nets.size();
    PhysNet net{id, name, cell_ids, {}};
    
    // ← Auto-generate pin offsets at cell centers
    for (auto cid : cell_ids) {
        net.pin_offsets.push_back({cells[cid].width / 2, 
                                   cells[cid].height / 2});
    }
    nets.push_back(net);
    return id;
}
```

### Implementation: total_wirelength() (Lines 26-42)

Half-Perimeter Wirelength (HPWL) — standard metric:

```cpp
double PhysicalDesign::total_wirelength() const {
    double total = 0;
    
    for (auto& net : nets) {
        if (net.cell_ids.size() < 2) continue;  // ← Skip single-cell nets
        
        // ← Find bounding box
        double min_x = 1e18, max_x = -1e18, min_y = 1e18, max_y = -1e18;
        for (size_t i = 0; i < net.cell_ids.size(); ++i) {
            auto& c = cells[net.cell_ids[i]];
            // ← Pin position = cell position + offset
            double px = c.position.x + (i < net.pin_offsets.size() ? 
                                        net.pin_offsets[i].x : 0);
            double py = c.position.y + (i < net.pin_offsets.size() ? 
                                        net.pin_offsets[i].y : 0);
            min_x = std::min(min_x, px); max_x = std::max(max_x, px);
            min_y = std::min(min_y, py); max_y = std::max(max_y, py);
        }
        // ← HPWL = half perimeter of bounding box
        total += (max_x - min_x) + (max_y - min_y);
    }
    return total;
}
```

**Formula**: HPWL = (max_x - min_x) + (max_y - min_y)

### Implementation: utilization() (Lines 44-49)

```cpp
double PhysicalDesign::utilization() const {
    double cell_area = 0;
    for (auto& c : cells) 
        cell_area += c.width * c.height;
    
    double die = die_area.area();
    return die > 0 ? cell_area / die : 0;
}
```

### Implementation: has_overlaps() (Lines 51-66)

O(n²) check for overlapping placed cells:

```cpp
bool PhysicalDesign::has_overlaps() const {
    for (size_t i = 0; i < cells.size(); ++i) {
        if (!cells[i].placed) continue;
        
        // ← Bounding box of cell i
        Rect ri(cells[i].position.x, cells[i].position.y,
                cells[i].position.x + cells[i].width,
                cells[i].position.y + cells[i].height);
        
        // ← Check against all higher-indexed cells
        for (size_t j = i + 1; j < cells.size(); ++j) {
            if (!cells[j].placed) continue;
            
            Rect rj(cells[j].position.x, cells[j].position.y,
                    cells[j].position.x + cells[j].width,
                    cells[j].position.y + cells[j].height);
            
            if (ri.overlaps(rj)) return true;
        }
    }
    return false;
}
```

---

## 📊 INTEGRATION DIAGRAM

```
    DEF File (text)
         ↓
    DefParser::parse_file()
         ↓
    tokenize() → vector<string>
         ↓
    parse_tokens() ← STATE MACHINE
         ├─ DIEAREA → die_area: Rect
         ├─ COMPONENTS → cells: CellInstance[]
         │   ├─ name, type
         │   ├─ position (x, y) [converted DBU→um]
         │   ├─ width=3.0, height=row_height [HARDCODED]
         │   └─ placed=true
         └─ NETS → nets: PhysNet[]
              ├─ name
              ├─ cell_ids: int[] [linear search lookup]
              └─ pin_offsets: Point[] [default: cell centers]
         ↓
    PhysicalDesign
         ├─ die_area: Rect
         ├─ cells: CellInstance[]
         └─ nets: PhysNet[]
         
    [Separate]
    
    Netlist (from Verilog parser)
         ├─ nets_: Net[] (id, name, value, fanout, driver)
         ├─ gates_: Gate[] (id, type, inputs, output, clk, reset)
         ├─ pis_, pos_: NetId[] (primary I/O)
         └─ dffs_: GateId[] (sequential elements)
         
    [Together]
    
    HierarchyManager
         ├─ modules_: map<name, DesignModule>
         │   ├─ name: string
         │   ├─ netlist: Netlist
         │   ├─ physical: PhysicalDesign
         │   ├─ is_leaf: bool
         │   ├─ sub_modules: string[] (child names)
         │   ├─ area_um2: double
         │   └─ gate_count: int
         ├─ add_module() → DesignModule
         ├─ instantiate(parent, child) → is_leaf=false
         ├─ analyze() → HierarchyResult
         ├─ flatten(top) → single Netlist
         └─ hierarchy_tree() → ASCII tree
```

---

## 🎯 KEY INSIGHTS

### 1. **Separation of Concerns**
- **Netlist**: Pure logic, no physical
- **PhysicalDesign**: Pure placement, no gates
- **DesignModule**: Bridges both layers

### 2. **Hierarchy Levels**
- **HierarchyManager**: Module instantiation (parent→child)
- **Netlist**: Always flat (no hierarchical structure)
- **flatten()**: Merges all modules into single netlist with hierarchical naming

### 3. **DEF Parser Characteristics**
- **Stateless**: No internal state
- **Selective**: Only DIEAREA, COMPONENTS, NETS
- **Hardcoded defaults**: cell width=3.0, height=row_height
- **No optimization**: Linear search for cell name lookup in NETS section
- **Minimal validation**: No duplicate checks, no DRC

### 4. **4-State Logic System**
- **Values**: 0, 1, X (unknown), Z (high-impedance)
- **Gate truth tables**: Implement IEEE 1364 semantics
- **Dominance rules**:
  - AND: 0 dominates
  - OR: 1 dominates
  - XOR: X if any unknown

### 5. **Connectivity**
- **Bidirectional**: nets → drivers/fanout, gates → inputs/output
- **No backedges**: Netlist is DAG except through DFFs
- **DFFs break loops**: topo_order() treats DFF output as source

### 6. **Limitations**
- ✗ No instance name tracking (only module names)
- ✗ No LEF parsing (cell definitions external)
- ✗ No hierarchical netlists (only flat)
- ✗ No routing implementation (wires/vias are stubs)
- ✗ No DRC checking
- ✗ Hardcoded cell dimensions in DEF parser

---

## 📝 CONCLUSIONS

**SiliconForge is a two-layer system**:

1. **Logical Layer** (Netlist class):
   - Gate-level combinational + sequential logic
   - No modules—always flat
   - Used for simulation and verification

2. **Physical Layer** (PhysicalDesign class):
   - Cell placement, nets, routing (stubs)
   - No gate semantics
   - Supports metrics: HPWL, utilization, overlap detection

3. **Hierarchy Manager** (HierarchyManager class):
   - Manages module instantiation (parent → child references)
   - Can flatten either layer
   - Tracks depth, counts gates/area

4. **DEF Parser** (DefParser class):
   - Converts DEF file → PhysicalDesign
   - Only parses DIEAREA, COMPONENTS, NETS
   - Minimal semantic support (no hierarchical DEF)

**The parser is intentionally simple**—it extracts placement and connectivity information for physical design, not gate-level logic. Gate definitions come from separate netlists (e.g., Verilog). This separation allows independent evolution of logical and physical representations within the hierarchy.

