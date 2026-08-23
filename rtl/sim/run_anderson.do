vlib an
vlog -work an ../fp64.v ../fp64_rsqrt.v ../fp64_cmp.v ../aa_gram.v ../chol10.v ../anderson.v ../tb_anderson.v
vsim -c an.tb_anderson
run -all
quit -f
