vlib ag4
vlog -work ag4 fp64.v fp64_cmp.v aa_gram.v tb_aa_gram.v
vsim -c ag4.tb_aa_gram
run -all
quit -f
