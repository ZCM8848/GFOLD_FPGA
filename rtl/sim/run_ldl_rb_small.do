vlib ldlrb
vlog -work ldlrb ../fp64.v ../banded_ldl_fp64_rb.v ../tb_ldl_rb.v
vsim -c -gWHICH=0 -gN=8 -gHB=2 ldlrb.tb_ldl_rb
run -all
quit -f
