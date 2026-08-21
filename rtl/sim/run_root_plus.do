vlib rp
vlog -work rp fp64.v fp64_rsqrt.v fp64_cmp.v root_plus.v tb_root_plus.v
vsim -c rp.tb_root_plus
run -all
quit -f
