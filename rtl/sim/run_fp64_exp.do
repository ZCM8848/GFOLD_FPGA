vlib expf
vlog -work expf ../fp64.v ../fp64_log.v ../fp64_exp.v ../fp64_exp_tb.v
vsim -c expf.tb_fp64_exp
run -all
quit -f
