vlib drsi
vlog -work drsi ../fp64.v ../fp64_rsqrt.v ../fp64_cmp.v ../proj_soc.v ../root_plus.v \
    ../aa_gram.v ../chol10.v ../anderson.v \
    ../spmv_fp64.v ../banded_ldl_fp64_rb.v ../kkt_solve.v ../proj_dual_cone_rb.v \
    ../drs_iter.v ../tb_drs_iter.v
vsim -c drsi.tb_drs_iter
add list -decimal /tb_drs_iter/dut/st /tb_drs_iter/dut/i /tb_drs_iter/dut/own_addr /tb_drs_iter/dut/own_we /tb_drs_iter/dut/vv /tb_drs_iter/dut/sqacc /tb_drs_iter/ram_addr /tb_drs_iter/ram_rdata
run 14784000000ns
write list -window .list nm_wave.txt
run -all
quit -f
