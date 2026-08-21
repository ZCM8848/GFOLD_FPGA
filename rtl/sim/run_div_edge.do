vlib edge
vlog -work edge ../float32.v ../tb_div_edge.v
vsim -c edge.tb_div_edge
run -all
quit -f
