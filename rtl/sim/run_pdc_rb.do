vlib pdcrb
vlog -work pdcrb ../fp64.v ../fp64_rsqrt.v ../fp64_cmp.v ../proj_soc.v ../proj_dual_cone_rb.v ../tb_proj_dual_cone_rb.v
vsim -c pdcrb.tb_proj_dual_cone_rb
run -all
quit -f
