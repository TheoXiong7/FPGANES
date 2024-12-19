set_property SRC_FILE_INFO {cfile:c:/Users/theox/Desktop/NES_Urbana_777/NES_Urbana_777.gen/sources_1/bd/mb_snes/ip/mb_snes_clk_wiz_1_0/mb_snes_clk_wiz_1_0.xdc rfile:../../../NES_Urbana_777.gen/sources_1/bd/mb_snes/ip/mb_snes_clk_wiz_1_0/mb_snes_clk_wiz_1_0.xdc id:1 order:EARLY scoped_inst:inst} [current_design]
current_instance inst
set_property src_info {type:SCOPED_XDC file:1 line:57 export:INPUT save:INPUT read:READ} [current_design]
set_input_jitter [get_clocks -of_objects [get_ports clk_in1]] 0.100
