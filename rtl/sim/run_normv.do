vlib nv
vlog -work nv ../fp64.v ../fp64_rsqrt.v ../normalize_v.v ../tb_normalize_v.v
vsim -c nv.tb_normalize_v
run -all
quit -f
