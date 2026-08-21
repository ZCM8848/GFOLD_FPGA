vlib pdcf
vlog -work pdcf fp64.v fp64_rsqrt.v fp64_cmp.v proj_soc.v proj_dual_cone.v tb_proj_dual_cone.v
vsim -c pdcf.tb_proj_dual_cone -gM=2107 -gINIT_FILE="data/pdc_full_init.hex" -gOUT_FILE="data/pdc_full_out.hex"
run -all
quit -f
