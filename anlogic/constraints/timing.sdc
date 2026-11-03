# gfold_m1 时序约束
# PLL: 50MHz → C0 125MHz（C0 = (FIN/REFCLK_DIV) × FBCLK_DIV = 50/2 × 5）

create_clock -name clk_in -period 20.000 -waveform {0 10} [get_ports {I_sys_clk}]

create_generated_clock -name clk_125 -source [get_ports {I_sys_clk}] -master_clock {clk_in} \
    -multiply_by 2.5 [get_pins {u_pll/ph1p_phy_pll_wrapper_25a56e5ce2f9_Inst/u_PH1P_PHY_PLL.clkc[0]}]

# solver clock: PLL C1 = 23.809524 MHz (period 42 ns), buffered by u_bufg_sol
create_clock -name clk_sol -period 42.000 -waveform {0 21.000} \
    [get_pins {u_bufg_sol/o}]

# PHY 送来的 RGMII RX_CLK（125MHz）
create_clock -name rgmii_rxc -period 8.000 -waveform {0 4} [get_ports {I_rgmii_rxc}]

# 三个时钟域互相异步；跨域只走单比特 toggle 握手
set_clock_groups -asynchronous \
    -group [get_clocks {clk_in}] \
    -group [get_clocks {clk_125}] \
    -group [get_clocks {rgmii_rxc}] \
    -group [get_clocks {clk_sol}]
