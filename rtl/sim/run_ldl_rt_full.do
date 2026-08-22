vlib ldlrtf
vlog -work ldlrtf ../fp64.v ../fp64_rsqrt.v ../banded_ldl_fp64_rt.v ../tb_ldl_rt.v
vsim -c -gWHICH=1 -gN=1100 -gHB=17 -gBAND_FILE=../data/full_f64/band_f64.hex ldlrtf.tb_ldl_rt
run -all
quit -f
