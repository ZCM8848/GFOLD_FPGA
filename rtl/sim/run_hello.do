# ModelSim do-file for the hello flow check. Run from rtl/sim/.
vlib work
vlog ../hello.v ../tb_hello.v
vsim -c work.tb_hello
run -all
quit -f
