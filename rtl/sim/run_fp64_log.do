vlib logf
vlog -work logf ../fp64.v ../fp64_log.v ../fp64_log_tb.v
vsim -c logf.tb_fp64_log
run -all
quit -f
