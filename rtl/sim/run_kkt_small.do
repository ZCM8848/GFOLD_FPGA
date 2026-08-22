vlib kkts
vlog -work kkts ../fp64.v ../spmv_fp64.v ../banded_ldl_fp64_rt.v ../kkt_solve.v ../tb_kkt_solve.v
vsim -c -gWHICH=0 -gN=10 -gM=20 -gNNZ=41 -gHB=4 kkts.tb_kkt_solve
run -all
quit -f
