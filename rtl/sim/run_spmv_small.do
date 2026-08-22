vlib spmvl
vlog -work spmvl ../fp64.v ../spmv_fp64.v ../tb_spmv.v
vsim -c -gN=10 -gM=20 -gNNZ=41 spmvl.tb_spmv
run -all
quit -f
