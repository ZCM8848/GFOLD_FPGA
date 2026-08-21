vlib an32
vlog -work an32 fp64.v fp64_rsqrt.v fp64_cmp.v aa_gram.v chol10.v anderson.v tb_anderson.v
vsim -c -gDIM=32 an32.tb_anderson
run -all
quit -f
