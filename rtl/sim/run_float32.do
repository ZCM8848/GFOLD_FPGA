# float32 unit test. Run from rtl/sim/
vlib work
vlog ../float32.v ../tb_float32.v
vsim -c work.tb_float32
run -all
quit -f
