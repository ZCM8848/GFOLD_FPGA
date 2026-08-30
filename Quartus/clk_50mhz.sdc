# 50 MHz oscillator on the DE2-115 (PIN_Y2) -> PLL -> 30 MHz solver clock
create_clock -name CLOCK_50 -period 20.000 [get_ports {CLOCK_50}]
derive_pll_clocks
derive_clock_uncertainty
