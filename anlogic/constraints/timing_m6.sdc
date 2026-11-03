create_clock -name clk_in -period 20.000 -waveform {0 10} [get_ports {I_sys_clk}]
create_clock -name clk_sol -period 40.000 -waveform {0 20} [get_pins {u_bufg_sol/o}]
set_clock_groups -asynchronous -group {clk_in} -group {clk_sol}
