vlib sbs
vlog -work sbs ../fp64.v ../s_build.v ../tb_s_build.v
vsim -c -gWHICH=0 -gN=10 -gM=20 -gNNZ=41 -gHB=4 -gMAXROW=4 sbs.tb_s_build
run -all
quit -f
