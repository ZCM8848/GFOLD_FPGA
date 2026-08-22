vlib kktf
vlog -work kktf ../fp64.v ../spmv_fp64.v ../banded_ldl_fp64_rt.v ../kkt_solve.v ../tb_kkt_solve.v
vsim -c -gWHICH=1 -gN=1100 -gM=2107 -gNNZ=4783 -gHB=17 \
    -gAROW_FILE=../data/kkt/full/Arow.hex -gACOL_FILE=../data/kkt/full/Acol.hex \
    -gAVAL_FILE=../data/kkt/full/Aval.hex -gRY_FILE=../data/kkt/full/r_y.hex \
    -gDY_FILE=../data/kkt/full/Dy.hex -gBAND_FILE=../data/kkt/full/band_f64.hex \
    kktf.tb_kkt_solve
run -all
quit -f
