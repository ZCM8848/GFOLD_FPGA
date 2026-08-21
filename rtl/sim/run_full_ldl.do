# Full-size (1100x1100, hb=17) banded LDL test. Run from rtl/sim/
# Pick scale by editing the -g BAND/RHS/EXP lines (keys: s1, s0_1, s1e-4).
vdel -all -lib full
vlib full
vlog -work full ../float32.v ../banded_ldl.v ../tb_banded_ldl.v
vsim -c full.tb_banded_ldl \
    -gN=1100 -gHB=17 \
    -gBAND="../data/full/band_s1.hex" \
    -gRHS="../data/full/rhs_s1.hex" \
    -gEXP="../data/full/exp_zx_s1.hex"
run -all
quit -f
