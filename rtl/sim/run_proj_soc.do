vlib psoc
vlog -work psoc ../fp64.v ../fp64_rsqrt.v ../fp64_cmp.v ../proj_soc.v ../tb_proj_soc.v
vsim -c psoc.tb_proj_soc
run -all
quit -f
