read_verilog studio/test_design.v
synth
floorplan 200 200 10
place
route
drc
lvs
sta
power
export_full test_out.json
exit
