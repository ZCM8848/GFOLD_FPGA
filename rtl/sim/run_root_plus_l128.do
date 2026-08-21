vlib rp128
vlog -work rp128 fp64.v fp64_rsqrt.v fp64_cmp.v root_plus.v tb_root_plus.v
vsim -c rp128.tb_root_plus -gL=128
run -all
quit -f
