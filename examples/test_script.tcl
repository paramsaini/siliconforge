read_verilog test_design.v
synth
floorplan 100 100 10
place
route
export_full test_state.json
exit
