vlib ch
vlog -work ch fp64.v fp64_rsqrt.v fp64_cmp.v chol10.v tb_chol10.v
vsim -c ch.tb_chol10
run -all
quit -f
