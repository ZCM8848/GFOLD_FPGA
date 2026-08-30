vlib drsi
vlog -work drsi ../fp64.v ../fp64_log.v ../fp64_exp.v ../fp64_rsqrt.v ../fp64_cmp.v ../proj_soc.v ../root_plus.v \
    ../s_build.v ../spmv_fp64.v ../banded_ldl_fp64_rb.v ../kkt_solve.v ../proj_dual_cone_rb.v \
    ../sram64_ctrl.v ../drs_iter.v ../tb_drs_iter.v
vsim -c drsi.tb_drs_iter
run -all
quit -f
