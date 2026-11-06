# Replay dataset ConnectorGuard-HDC di Kria KV260

Isi: `rtl/cg_replay_top.sv` (feeder + logger + core), `constraints/cg_replay.xdc`,
`notebook/cg_dataset_replay.ipynb` + `notebook/cg_replay.py`, `sim/` (testbench Verilator).

## Block design (Vivado), ganti cg_fpga_top/cg_ps
- Zynq UltraScale+ PS (preset kv260_som 1.4): pl_clk0 = 100 MHz, aktifkan M_AXI_HPM0_FPD dan S_AXI_HP0_FPD.
- Processor System Reset: slowest_sync_clk = pl_clk0, ext_reset_in = pl_resetn0.
- AXI DMA (axi_dma_0): Scatter Gather OFF, Write+Read channel ON, **Buffer Length Register width = 23**,
  **Address width = 40**, MM2S stream data width = **128**, S2MM stream data width = **256**.
- AXI GPIO (axi_gpio_0): dual channel; ch1 = 32 bit **All Outputs**, ch2 = 32 bit **All Inputs**.
- Module reference `cg_replay_top` (tambahkan semua file rtl.zip + cg_replay_top.sv; cg_pkg.sv paling atas):
  clk <- pl_clk0, rst_n <- peripheral_aresetn, ctrl <- axi_gpio_0/gpio_io_o, stat -> axi_gpio_0/gpio2_io_i,
  s_axis <- axi_dma_0/M_AXIS_MM2S, m_axis -> axi_dma_0/S_AXIS_S2MM.
- Make External untuk: contactor_close, state[1:0], hard_trip, cp_pwm, heartbeat (nama harus persis).
- AXI-Lite: SmartConnect dari M_AXI_HPM0_FPD ke axi_dma_0/S_AXI_LITE dan axi_gpio_0/S_AXI.
  Data DMA: M_AXI_MM2S dan M_AXI_S2MM lewat SmartConnect ke S_AXI_HP0_FPD.
- Constraint: `cg_replay.xdc`. Generate bitstream, ekspor `.bit` dan `.hwh`, namai `cg_replay.bit/.hwh`.

## Di Kria
Salin `cg_replay.bit`, `cg_replay.hwh`, `cg_replay.py`, `cg_dataset_replay.ipynb`, dan folder `dataset/`
ke satu folder Jupyter, buka notebook, Run All.

## Simulasi (sudah dijalankan: 716 window, 0 beda dari validation/rtl_output.csv)
`cd sim && RTL_DIR=<folder rtl> DATASET_DIR=<folder dataset> ./run_sim.sh`
lalu `cg_replay.compare(cg_replay.decode_log(<log.txt sebagai array uint32 (n,8)>), DATASET_DIR)`.
