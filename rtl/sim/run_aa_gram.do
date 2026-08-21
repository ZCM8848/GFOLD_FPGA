vlib ag
vlog -work ag fp64.v fp64_cmp.v aa_gram.v tb_aa_gram.v
vsim -c ag.tb_aa_gram
run -all
quit -f
