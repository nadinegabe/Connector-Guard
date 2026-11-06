## ConnectorGuard-HDC dataset replay on AMD Kria KV260. Pin Pmod sama dengan cg.xdc.
## Clock = pl_clk0 100 MHz dari PS (create_clock sudah ada di XDC IP PS, jangan diulang).
set_false_path -to [get_pins -of_objects [get_cells -hier {rst_sync_reg[*]}] -filter {REF_PIN_NAME == CLR}]

set_property -dict {PACKAGE_PIN E10 IOSTANDARD LVCMOS33 DRIVE 4 SLEW SLOW} [get_ports contactor_close]
set_property -dict {PACKAGE_PIN D10 IOSTANDARD LVCMOS33 DRIVE 4 SLEW SLOW} [get_ports {state[0]}]
set_property -dict {PACKAGE_PIN C11 IOSTANDARD LVCMOS33 DRIVE 4 SLEW SLOW} [get_ports {state[1]}]
set_property -dict {PACKAGE_PIN E12 IOSTANDARD LVCMOS33 DRIVE 4 SLEW SLOW} [get_ports hard_trip]
set_property -dict {PACKAGE_PIN D11 IOSTANDARD LVCMOS33 DRIVE 4 SLEW SLOW} [get_ports cp_pwm]
set_property -dict {PACKAGE_PIN B11 IOSTANDARD LVCMOS33 DRIVE 4 SLEW SLOW} [get_ports heartbeat]
