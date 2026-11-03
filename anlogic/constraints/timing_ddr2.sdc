# DDR2 smoke test clocks. The DDR2 IP supplies its own SDC (ddr2.sdc);
# this only constrains the 50 MHz board reference.
create_clock -name clk_in -period 20.000 -waveform {0 10} [get_ports {I_sys_clk}]
