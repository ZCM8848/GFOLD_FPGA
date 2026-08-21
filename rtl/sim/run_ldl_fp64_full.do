vlib f64f
vlog -work f64f ../fp64.v ../banded_ldl_fp64.v ../tb_banded_ldl_fp64.v
vsim -c f64f.tb_banded_ldl_fp64 -gN=1100 -gHB=17 \
    -gBAND="../data/full_f64/band_f64.hex" \
    -gRHS="../data/full_f64/rhs_f64.hex" \
    -gEXP="../data/full_f64/exp_zx_f64.hex"
run -all
quit -f
