vlib rsqrt
vlog -work rsqrt ../fp64.v ../fp64_rsqrt.v ../tb_fp64_rsqrt.v
vsim -c rsqrt.tb_fp64_rsqrt
run -all
quit -f
