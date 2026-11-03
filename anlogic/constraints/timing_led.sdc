# LED walk test reference clock.
create_clock -name clk_in -period 20.000 -waveform {0 10} [get_ports {I_sys_clk}]
