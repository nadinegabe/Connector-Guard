// ConnectorGuard-HDC dataset replay top for Kria KV260.
//
// Replaces cg_fpga_top (internal pattern generator) with a feeder that takes the
// dataset from the PS through an AXI DMA (AXI4-Stream, MM2S) and plays it into
// cg_top's live_data port at exactly one slot every PERIOD clocks (400 = 250 kS/s),
// the same cadence as tb_dataset.sv. Register-bus commands travel in the same stream
// and are written PERIOD/2 clocks after the preceding slot, as in the testbench.
// One 256-bit log beat per window leaves through AXI4-Stream (S2MM).
//
// Stimulus beat, 128 bit = 4 x uint32 (little endian):
//   kind (w3[1:0]) = 1 : sample slot   w0={v,i} w1={tamb,t} w2={16'b0,ires}
//   kind           = 0 : empty slot (ADC dropout), data ignored
//   kind           = 2 : bus write     w0=wdata  w1[7:0]=addr
// Log beat, 256 bit = 8 x uint32:
//   w0={win_idx[15:0], last_slot[15:0]}  w1=STATUS(0x01) w2=FEAT0(0x03) w3=FEAT1(0x04)
//   w4=HDC(0x05) w5=HEALTH(0x06) w6=clock counter  w7=0 (0xE0F0E0F0 on the end marker, w0=0xFFFFFFFF)
//
// ctrl (axi_gpio ch1 out): [0] go  [1] soft reset  [2] mirror_en  [13:8] mirror pins
//   mirror pins order = {heartbeat, cp_pwm, hard_trip, state[1], state[0], contactor_close}
// stat (axi_gpio ch2 in) : [0] core ready  [1] running  [2] done  [3] log overflow
//                          [4] last beat taken  [5] reset released  [15:6] window  [31:16] slot
module cg_replay_top
  import cg_pkg::*;
#(
  parameter int PERIOD     = 400,      // clocks per slot (>= 110 and >= 400 for the dataset)
  parameter int WIN_LOG2   = 6,        // must equal win_log2 written by the dataset commands
  parameter int PWM_PERIOD = 100000    // 100 MHz / 1 kHz control pilot
) (
  (* X_INTERFACE_PARAMETER = "ASSOCIATED_BUSIF s_axis:m_axis, ASSOCIATED_RESET rst_n" *)
  input  logic         clk,
  (* X_INTERFACE_PARAMETER = "POLARITY ACTIVE_LOW" *)
  input  logic         rst_n,
  input  logic [31:0]  ctrl,
  output logic [31:0]  stat,
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TDATA" *)  input  logic [127:0] s_axis_tdata,
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TVALID" *) input  logic         s_axis_tvalid,
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TREADY" *) output logic         s_axis_tready,
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 s_axis TLAST" *)  input  logic         s_axis_tlast,
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TDATA" *)  output logic [255:0] m_axis_tdata,
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TVALID" *) output logic         m_axis_tvalid,
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TREADY" *) input  logic         m_axis_tready,
  (* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 m_axis TLAST" *)  output logic         m_axis_tlast,
  // Pmod pins (same port names as cg.xdc)
  output logic         contactor_close,
  output logic [1:0]   state,
  output logic         hard_trip,
  output logic         cp_pwm,
  output logic         heartbeat
);
  // ---------------- reset: async assert, sync release; soft reset from ctrl[1] ----------------
  logic go, soft_rst, mirror_en;
  logic [5:0] mirror;
  assign go        = ctrl[0];
  assign soft_rst  = ctrl[1];
  assign mirror_en = ctrl[2];
  assign mirror    = ctrl[13:8];

  logic rst_all_n, rst_core_n;
  (* ASYNC_REG = "TRUE" *) logic [1:0] rst_sync;
  assign rst_all_n = rst_n & ~soft_rst;
  always_ff @(posedge clk or negedge rst_all_n) begin
    if (!rst_all_n) rst_sync <= 2'b00;
    else            rst_sync <= {rst_sync[0], 1'b1};
  end
  assign rst_core_n = rst_sync[1];

  // ---------------- core ----------------
  logic        live_valid, live_ready, rp_busy;
  sample_t     live_data;
  logic        bus_wr, bus_rd;
  logic [7:0]  bus_addr;
  logic [31:0] bus_wdata, bus_rdata;
  logic        c_close, c_hard, c_pwm;
  logic [1:0]  c_state;
  logic [7:0]  cp_duty, health;

  cg_top #(.PWM_PERIOD(PWM_PERIOD)) u_core (
    .clk, .rst_n(rst_core_n),
    .live_valid, .live_data, .live_ready,
    .src_sel(1'b0), .rp_wr_en(1'b0), .rp_wr_addr('0), .rp_wr_data('0),
    .rp_start(1'b0), .rp_len('0), .rp_period('0), .rp_busy,
    .bus_wr, .bus_rd, .bus_addr, .bus_wdata, .bus_rdata,
    .contactor_close(c_close), .cp_pwm(c_pwm), .cp_duty, .state(c_state),
    .hard_trip(c_hard), .health
  );

  // ---------------- bus mux ----------------
  logic        wr_q, rd_q;
  logic [7:0]  wr_addr_q, rd_addr_q;
  logic [31:0] wr_data_q;
  assign bus_wr    = wr_q;
  assign bus_rd    = rd_q;
  assign bus_addr  = wr_q ? wr_addr_q : rd_addr_q;
  assign bus_wdata = wr_data_q;

  // ---------------- read sequencer: STATUS, FEAT0, FEAT1, HDC, HEALTH ----------------
  logic [7:0]  rd_list [5];
  assign rd_list[0] = 8'h01; assign rd_list[1] = 8'h03; assign rd_list[2] = 8'h04;
  assign rd_list[3] = 8'h05; assign rd_list[4] = 8'h06;
  logic [31:0] cap [5];
  logic [3:0]  rq;
  logic        rd_run, rd_start, rd_done;

  always_ff @(posedge clk or negedge rst_core_n) begin
    if (!rst_core_n) begin
      rq <= '0; rd_run <= 1'b0; rd_done <= 1'b0; rd_q <= 1'b0; rd_addr_q <= 8'h01;
      for (int k = 0; k < 5; k++) cap[k] <= '0;
    end else begin
      rd_done <= 1'b0;
      if (!rd_run) begin
        if (rd_start) begin rd_run <= 1'b1; rq <= 4'd0; end
      end else begin
        rq <= rq + 1'b1;
        unique case (rq)
          4'd0:  begin rd_q <= 1'b1; rd_addr_q <= rd_list[0]; end
          4'd1:  rd_q <= 1'b0;
          4'd2:  begin cap[0] <= bus_rdata; rd_q <= 1'b1; rd_addr_q <= rd_list[1]; end
          4'd3:  rd_q <= 1'b0;
          4'd4:  begin cap[1] <= bus_rdata; rd_q <= 1'b1; rd_addr_q <= rd_list[2]; end
          4'd5:  rd_q <= 1'b0;
          4'd6:  begin cap[2] <= bus_rdata; rd_q <= 1'b1; rd_addr_q <= rd_list[3]; end
          4'd7:  rd_q <= 1'b0;
          4'd8:  begin cap[3] <= bus_rdata; rd_q <= 1'b1; rd_addr_q <= rd_list[4]; end
          4'd9:  rd_q <= 1'b0;
          4'd10: begin cap[4] <= bus_rdata; rd_run <= 1'b0; rd_done <= 1'b1; end
          default: ;
        endcase
      end
    end
  end

  // ---------------- feeder ----------------
  typedef enum logic [2:0] {F_RDY, F_FETCH, F_CMD, F_SLOT, F_DONE, F_END} f_t;
  f_t f_st;

  localparam int PW       = $clog2(PERIOD);
  localparam int CMD_AT   = PERIOD / 2;
  localparam int RD_AT    = PERIOD - 60;
  localparam logic [PW-1:0] LAST_PC = PW'(PERIOD - 1);

  logic [PW-1:0] pc;
  logic [15:0]   slot_idx;       // index of the slot that started last
  logic [15:0]   win_idx;
  logic [WIN_LOG2-1:0] vcount;
  logic          slots_started, win_flag, rdy_seen, last_q, done_q, ovf_q;
  logic [4:0]    rdy_t;
  logic [1:0]    kind_q;
  logic [31:0]   w0_q, w1_q, w2_q;
  logic [31:0]   cyc;
  logic          m_valid, m_last;
  logic [255:0]  m_data;
  logic          log_busy;

  assign s_axis_tready = (f_st == F_FETCH);
  logic win_start, poll_start;
  assign win_start  = (f_st != F_RDY) && win_flag && (pc == PW'(RD_AT));
  // while waiting for M5b/M6 to report ready, poll STATUS every 32 clocks
  assign poll_start = (f_st == F_RDY) && !rdy_seen && (rdy_t == 5'd0);
  assign rd_start   = win_start | poll_start;

  always_ff @(posedge clk or negedge rst_core_n) begin
    if (!rst_core_n) begin
      f_st <= F_RDY; pc <= '0; slot_idx <= '1; win_idx <= '0; vcount <= '0;
      slots_started <= 1'b0; win_flag <= 1'b0; rdy_seen <= 1'b0; last_q <= 1'b0;
      done_q <= 1'b0; ovf_q <= 1'b0; rdy_t <= '0; kind_q <= '0;
      w0_q <= '0; w1_q <= '0; w2_q <= '0; cyc <= '0;
      m_valid <= 1'b0; m_last <= 1'b0; m_data <= '0;
      live_valid <= 1'b0; live_data <= '0; wr_q <= 1'b0; wr_addr_q <= '0; wr_data_q <= '0;
    end else begin
      cyc        <= cyc + 1'b1;
      pc         <= (pc == LAST_PC) ? '0 : pc + 1'b1;
      live_valid <= 1'b0;
      wr_q       <= 1'b0;
      if (m_valid && m_axis_tready) begin m_valid <= 1'b0; m_last <= 1'b0; end

      // window log push once the read sequence has finished
      if (rd_done && win_flag) begin
        win_flag <= 1'b0;
        win_idx  <= win_idx + 1'b1;
        if (m_valid && !m_axis_tready) ovf_q <= 1'b1;
        m_valid <= 1'b1;
        m_last  <= 1'b0;
        m_data  <= {32'd0, cyc, cap[4], cap[3], cap[2], cap[1], cap[0], win_idx + 16'd1, slot_idx};
      end

      unique case (f_st)
        F_RDY: begin
          rdy_t <= rdy_t + 1'b1;
          if (rd_done && cap[0][12:11] == 2'b11) rdy_seen <= 1'b1;
          if (rdy_seen && go && !rd_run) f_st <= F_FETCH;
        end
        F_FETCH: begin
          if (s_axis_tvalid) begin
            w0_q <= s_axis_tdata[31:0];   w1_q <= s_axis_tdata[63:32];
            w2_q <= s_axis_tdata[95:64];  kind_q <= s_axis_tdata[97:96];
            last_q <= s_axis_tlast;
            f_st <= (s_axis_tdata[97:96] == 2'd2) ? F_CMD : F_SLOT;
          end
        end
        F_CMD: begin
          // before the first slot: write right away; later: PERIOD/2 into the slot
          if (!slots_started || pc >= PW'(CMD_AT)) begin
            wr_q <= 1'b1; wr_addr_q <= w1_q[7:0]; wr_data_q <= w0_q;
            f_st <= last_q ? F_DONE : F_FETCH;
          end
        end
        F_SLOT: begin
          if (pc == LAST_PC) begin
            live_valid    <= kind_q[0];
            live_data.i   <= w0_q[15:0];  live_data.v    <= w0_q[31:16];
            live_data.t   <= w1_q[15:0];  live_data.tamb <= w1_q[31:16];
            live_data.ires<= w2_q[15:0];
            slot_idx      <= slot_idx + 1'b1;
            slots_started <= 1'b1;
            if (kind_q[0]) begin
              vcount <= vcount + 1'b1;
              if (vcount == WIN_LOG2'((1 << WIN_LOG2) - 1)) win_flag <= 1'b1;
            end
            f_st <= last_q ? F_DONE : F_FETCH;
          end
        end
        F_DONE: begin
          // let the last slot (and its window log) finish, then send the end marker
          if (pc == LAST_PC && !win_flag && !rd_run && !m_valid) begin
            m_valid <= 1'b1; m_last <= 1'b1;
            m_data  <= {32'hE0F0E0F0, cyc, 32'd0, 32'd0, 32'd0, 32'd0, 32'd0, 32'hFFFFFFFF};
            f_st    <= F_END;
          end
        end
        F_END: if (!m_valid) done_q <= 1'b1;
        default: ;
      endcase
    end
  end

  assign m_axis_tdata  = m_data;
  assign m_axis_tvalid = m_valid;
  assign m_axis_tlast  = m_last;

  // ---------------- status ----------------
  assign stat = {slot_idx, win_idx[9:0], rst_core_n, last_q, ovf_q, done_q, (f_st != F_RDY), rdy_seen};

  // ---------------- Pmod outputs ----------------
  logic [26:0] blink;
  always_ff @(posedge clk or negedge rst_core_n)
    if (!rst_core_n) blink <= '0; else blink <= blink + 1'b1;

  logic [5:0] pins_core, pins;
  assign pins_core = {blink[26], c_pwm, c_hard, c_state[1], c_state[0], c_close};
  assign pins      = mirror_en ? mirror : pins_core;
  assign contactor_close = pins[0];
  assign state           = pins[2:1];
  assign hard_trip       = pins[3];
  assign cp_pwm          = pins[4];
  assign heartbeat       = pins[5];
endmodule
