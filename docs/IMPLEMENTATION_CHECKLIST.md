# Implementation Checklist for ssta.cpp

## ✅ Phase 1: Pre-Implementation (COMPLETE)

- [x] Directory structure mapped (16 subsystems, 70+ files)
- [x] CMakeLists.txt build system understood
- [x] sta.hpp/cpp (852 lines) fully analyzed
- [x] StaEngine public/private interfaces documented
- [x] Netlist connectivity model understood
- [x] LibertyLibrary timing data structures known
- [x] OCV derating modes (NONE/OCV/AOCV/POCV) explained
- [x] Test pattern (test_phase38.cpp) analyzed
- [x] Build configuration for CMake verified

## ✅ Phase 2: Documentation (COMPLETE)

### Generated Files (4 documents, 1,801 lines)

- [x] **SILICONFORGE_ANALYSIS.md** (725 lines)
  - Project structure and subsystems
  - Full CMakeLists.txt files
  - sta.hpp complete (307 lines)
  - sta.cpp breakdown (852 lines) with all methods
  - Test patterns and integration guide

- [x] **FULL_FILE_CONTENTS.md** (547 lines)
  - Complete file excerpts
  - Key method implementations
  - Full class definitions
  - CMake configurations
  - Test structure examples

- [x] **QUICK_REFERENCE.md** (377 lines)
  - 3-step guide for adding ssta.cpp
  - StaEngine API reference
  - Netlist interface with examples
  - LibertyLibrary usage
  - OCV modes explanation
  - test_phase39.cpp template
  - Build commands

- [x] **README.md** (152 lines)
  - Project overview
  - STA capabilities
  - Key interfaces
  - Design patterns
  - References and locations

## ⏭️ Phase 3: Implementation (READY)

### File: src/timing/ssta.hpp (to create)

- [ ] Create `/home/paramsaini/Desktop/siliconforge/src/timing/ssta.hpp`
- [ ] Add include guards: `#pragma once`
- [ ] Include dependencies:
  ```cpp
  #include "timing/sta.hpp"
  #include <vector>
  #include <unordered_map>
  ```
- [ ] Declare `class SstaEngine`
- [ ] Constructor: `SstaEngine(const Netlist&, const LibertyLibrary*, const PhysicalDesign*)`
- [ ] Public methods:
  - [ ] `StaResult analyze_statistical(double clock_period, int num_paths = 5)`
  - [ ] `std::vector<StaResult> analyze_corners_with_distribution(double)`
  - [ ] `void set_distribution_parameters(...)`
  - [ ] Configuration methods for statistical parameters
- [ ] Private methods:
  - [ ] Statistical derating computations
  - [ ] Corner distribution functions
  - [ ] Confidence interval calculations
  - [ ] Path sigma aggregation
- [ ] Private members:
  - [ ] `StaEngine sta_` (wrapped instance)
  - [ ] Distribution parameters
  - [ ] Statistical state

### File: src/timing/ssta.cpp (to create)

- [ ] Implementation file with include: `#include "timing/ssta.hpp"`
- [ ] Constructor implementation
- [ ] `analyze_statistical()` method:
  - [ ] Call `sta.analyze()` for nominal corner
  - [ ] Add path-level sigma computation
  - [ ] Apply statistical derating adjustments
  - [ ] Return augmented StaResult
- [ ] `analyze_corners_with_distribution()` method:
  - [ ] Call `sta.analyze_multicorner()` for 3 base corners
  - [ ] Generate additional statistical corners
  - [ ] Compute corner distributions
  - [ ] Return vector of corner-specific results
- [ ] Helper methods for statistical computations
- [ ] Integration with existing OCV infrastructure

### File: src/CMakeLists.txt (to modify)

- [ ] Open `/home/paramsaini/Desktop/siliconforge/src/CMakeLists.txt`
- [ ] Find "Timing & Power" section (line 73)
- [ ] Add line after `timing/sta.cpp`:
  ```cmake
  timing/ssta.cpp
  ```
- [ ] Result should read:
  ```cmake
  # ── Timing & Power ────────────────────────────────────────────
  timing/sta.cpp
  timing/ssta.cpp              # ← NEW LINE
  timing/power.cpp
  ```
- [ ] Verify no syntax errors
- [ ] Save file

### File: tests/CMakeLists.txt (to modify)

- [ ] Open `/home/paramsaini/Desktop/siliconforge/tests/CMakeLists.txt`
- [ ] Find last test entry (test_phase38 at line 149-151)
- [ ] Add after line 151:
  ```cmake
  add_executable(test_phase39 test_phase39.cpp)
  target_link_libraries(test_phase39 PRIVATE siliconforge_lib)
  add_test(NAME Phase39Tests COMMAND test_phase39)
  ```
- [ ] Save file

### File: tests/test_phase39.cpp (to create)

- [ ] Create `/home/paramsaini/Desktop/siliconforge/tests/test_phase39.cpp`
- [ ] Use template from QUICK_REFERENCE.md
- [ ] Include headers:
  - [ ] `#include "core/netlist.hpp"`
  - [ ] `#include "timing/sta.hpp"`
  - [ ] `#include "timing/ssta.hpp"` (when ready)
- [ ] Define test infrastructure (using macro pattern):
  - [ ] `TEST()` macro wrapper
  - [ ] `CHECK()` assertion macro
  - [ ] `PASS()` counter macro
  - [ ] `RUN()` test runner macro
- [ ] Implement tests:
  - [ ] `test_build_netlist()` — Basic netlist creation
  - [ ] `test_sta_basic()` — Basic STA analysis
  - [ ] `test_sta_with_aocv()` — AOCV derating
  - [ ] `test_ssta_basic()` — SSTA statistical analysis (phase39-specific)
  - [ ] `test_ssta_distribution()` — Corner distribution analysis
  - [ ] `test_ssta_path_sigma()` — Path-level sigma computation
  - [ ] Additional tests as needed
- [ ] Implement `main()`:
  - [ ] Run all tests with RUN() macro
  - [ ] Print summary: Passed/Failed counts
  - [ ] Return 0 if all pass, 1 if any fail

## ⏭️ Phase 4: Build & Test

- [ ] Navigate to build directory:
  ```bash
  cd ~/Desktop/siliconforge/build
  ```
- [ ] Run CMake:
  ```bash
  cmake ..
  ```
- [ ] Verify CMakeLists.txt changes are detected
- [ ] Build project:
  ```bash
  make
  ```
- [ ] Verify ssta.cpp compiled without errors
- [ ] Verify test_phase39 executable created
- [ ] Run specific test:
  ```bash
  ./tests/test_phase39
  ```
- [ ] Verify test output shows all PASS entries
- [ ] Run all tests:
  ```bash
  ctest
  ```
- [ ] Verify no regressions in existing tests

## ⏭️ Phase 5: Validation

- [ ] Verify ssta.hpp compiles standalone
- [ ] Verify ssta.cpp links with siliconforge_lib
- [ ] Verify test_phase39 runs independently
- [ ] Verify all test assertions pass
- [ ] Verify integration with existing StaEngine
- [ ] Check for memory leaks (valgrind if available)
- [ ] Verify multi-corner analysis works correctly
- [ ] Verify OCV derating layers work (NONE/OCV/AOCV/POCV)
- [ ] Test with minimal and complex netlists
- [ ] Document any discoveries in comments

## 📊 Success Criteria

- [x] Documentation complete (4 files, 1,801 lines analyzed)
- [ ] ssta.hpp created and compiles
- [ ] ssta.cpp created and links successfully
- [ ] CMakeLists.txt updated for both src/ and tests/
- [ ] test_phase39.cpp created and passes
- [ ] All 39 tests pass (test_phase1..39)
- [ ] No compilation warnings or errors
- [ ] API consistent with StaEngine patterns
- [ ] Code follows project style (C++20, sf namespace)
- [ ] Comments document key statistical methods

## 🔗 Reference Materials

- **SILICONFORGE_ANALYSIS.md** — Technical deep-dive
- **FULL_FILE_CONTENTS.md** — Complete file listings
- **QUICK_REFERENCE.md** — API and pattern templates
- **README.md** — Project overview
- **IMPLEMENTATION_CHECKLIST.md** — This file

## 🏁 Notes

1. **Building incrementally**: Start with stub ssta.hpp/cpp that just wrap StaEngine
2. **Test-driven**: Write tests BEFORE statistical features
3. **Dependency management**: ssta only depends on existing sta.hpp/cpp
4. **CMake ordering**: Timing section must stay after core/frontend, before verify
5. **Namespace**: Must be in `namespace sf { }` like all other code
6. **Style**: Follow existing code style (see sta.cpp for examples)
7. **Error handling**: Return meaningful messages in StaResult.message
8. **OCV integration**: Leverage existing OcvMode enum and derating infrastructure

