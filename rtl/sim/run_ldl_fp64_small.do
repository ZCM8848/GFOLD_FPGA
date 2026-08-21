vlib f64s
vlog -work f64s ../fp64.v ../banded_ldl_fp64.v ../tb_banded_ldl_fp64.v
vsim -c f64s.tb_banded_ldl_fp64 -gN=8 -gHB=2 \
    -gBAND="../data/small_f64/band_f64.hex" \
    -gRHS="../data/small_f64/rhs_f64.hex" \
    -gEXP="../data/small_f64/exp_zx_f64.hex"
run -all
quit -f
