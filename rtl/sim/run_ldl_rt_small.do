vlib ldlrt
vlog -work ldlrt ../fp64.v ../fp64_rsqrt.v ../banded_ldl_fp64_rt.v ../tb_ldl_rt.v
vsim -c -gWHICH=0 -gN=8 -gHB=2 -gBAND_FILE=../data/small_f64/band_f64.hex ldlrt.tb_ldl_rt
run -all
quit -f
