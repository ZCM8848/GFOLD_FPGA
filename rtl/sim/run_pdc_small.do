vlib pdcs
vlog -work pdcs fp64.v fp64_rsqrt.v fp64_cmp.v proj_soc.v proj_dual_cone.v tb_proj_dual_cone.v
vsim -c pdcs.tb_proj_dual_cone -gM=18 -gZ=2 -gNONNEG=2 -gN=2 -gSOC_START=4 -gN4=2 -gDIM4=4 -gN3=2 -gDIM3=3 -gINIT_FILE="data/pdc_small_init.hex" -gOUT_FILE="data/pdc_small_out.hex"
run -all
quit -f
