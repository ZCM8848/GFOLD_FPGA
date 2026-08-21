# banded LDL test. Run from rtl/sim/
vlib work
vlog ../float32.v ../banded_ldl.v ../tb_banded_ldl.v
vsim -c work.tb_banded_ldl
run -all
quit -f
