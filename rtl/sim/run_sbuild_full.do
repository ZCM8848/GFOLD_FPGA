vlib sbsf
vlog -work sbsf ../fp64.v ../s_build.v ../tb_s_build.v
vsim -c -gWHICH=1 -gN=1100 -gM=2107 -gNNZ=4783 -gHB=17 -gMAXROW=6 \
    -gAROW_FILE=../data/kkt/full/Arow.hex -gACOL_FILE=../data/kkt/full/Acol.hex \
    -gAVAL_FILE=../data/kkt/full/Aval.hex sbsf.tb_s_build
run -all
quit -f
