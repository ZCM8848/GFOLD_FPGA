vlib ldlrbf
vlog -work ldlrbf ../fp64.v ../banded_ldl_fp64_rb.v ../tb_ldl_rb.v
vsim -c -gWHICH=1 -gN=1100 -gHB=17 ldlrbf.tb_ldl_rb
run -all
quit -f
