// M9: register file, configuration lock, commands and output interface.
//
// Bus: single-cycle write (bus_wr); read data is registered one cycle after
// bus_rd. Word addresses:
//   0x00 ID        RO  0x43474844 ("CGHD")
//   0x01 STATUS    RO  see README
//   0x02 CMD       WO  bit0 LOCK_CFG, bit1 TRIP_CLEAR, bit2 BASELINE_RESET,
//                      bit3 CLS_RESET, bit4 TRAIN_START (label in [9:8]),
//                      bit5 TRAIN_STOP
//   0x03 FEAT0     RO  [15:0] r_comp, [31:16] th_idx
//   0x04 FEAT1     RO  [23:0] e_hf
//   0x05 HDC       RO  [15:0] d_adapt, [31:16] d_frozen
//   0x06 HEALTH    RO  [7:0] health index, [15:8] CP duty
//   0x10..0x1C     RW until LOCK_CFG, then read-only (writes set cfg_wr_err)
//
// Safety rules implemented here:
//   * contactor_close = cfg_locked & ~hard_trip & ~fsm_trip (combinational,
//     no register or firmware between M8 and the pin)
//   * BASELINE_RESET, CLS_RESET and TRAIN_START are accepted only while the
//     contactor is open; otherwise they are dropped and cmd_rej is set
module cg_m9_regs
  import cg_pkg::*;
#(
  parameter int PWM_PERIOD = 1000
) (
  input  logic        clk,
  input  logic        rst_n,
  // bus
  input  logic        bus_wr,
  input  logic        bus_rd,
  input  logic [7:0]  bus_addr,
  input  logic [31:0] bus_wdata,
  output logic [31:0] bus_rdata,
  // configuration / commands
  output cfg_t        cfg,
  output logic        cfg_locked,
  output logic        trip_clear,
  output logic        base_reset,
  output logic        cls_reset,
  output logic        train_active,
  output logic [1:0]  train_label,
  output logic        cfg_wr_err,
  output logic        cmd_rej,
  // status inputs
  input  logic [1:0]  state,
  input  logic        hard_trip,
  input  logic [4:0]  m8_cause,
  input  logic        fsm_trip,
  input  logic        sensor_fault,
  input  logic        stale,
  input  logic        arc,
  input  logic [1:0]  cls,
  input  logic [2:0]  cls_trained,
  input  logic        m5b_ready,
  input  logic        m6_ready,
  input  logic        m5b_overrun,
  input  logic        m6_overrun,
  input  logic [15:0] r_comp,
  input  logic [15:0] th_idx,
  input  logic [23:0] e_hf,
  input  logic [15:0] d_adapt,
  input  logic [15:0] d_frozen,
  // outputs
  output logic        contactor_close,
  output logic [7:0]  cp_duty,
  output logic        cp_pwm,
  output logic [7:0]  health
);
  localparam logic [31:0] ID = 32'h43474844;

  assign contactor_close = cfg_locked && !hard_trip && !fsm_trip;

  function automatic logic [31:0] cfg_read(input cfg_t c, input logic [7:0] a);
    unique case (a)
      8'h10: return {23'd0, c.learn_en, 4'd0, c.win_log2};
      8'h11: return {16'd0, c.stale_cycles};
      8'h12: return {c.di_max, c.i_max};
      8'h13: return {c.ires_max, c.t_max};
      8'h14: return {c.t_hi, c.t_lo};
      8'h15: return {8'd0, c.alpha_t, c.i_min_r};
      8'h16: return {8'd0, c.drift_shift, c.ewma_shift, c.tref};
      8'h17: return {c.hold_windows, c.mad_floor, c.k_derate, c.k_watch};
      8'h18: return {4'd0, c.arc_confirm, c.k_novel, c.forget_period, c.freeze_n};
      8'h19: return {c.cls_qsh, c.hdc_qsh};
      8'h1A: return {c.hdc_drift_thr, c.e_gate};
      8'h1B: return {c.tbin_thr, 12'd0, c.ibin_shift};
      8'h1C: return {16'd0, c.duty_derate, c.duty_normal};
      default: return 32'd0;
    endcase
  endfunction

  function automatic cfg_t cfg_write(input cfg_t c, input logic [7:0] a, input logic [31:0] d);
    cfg_t n;
    n = c;
    unique case (a)
      8'h10: begin n.win_log2 = d[3:0]; n.learn_en = d[8]; end
      8'h11: n.stale_cycles = d[15:0];
      8'h12: begin n.i_max = d[15:0]; n.di_max = d[31:16]; end
      8'h13: begin n.t_max = d[15:0]; n.ires_max = d[31:16]; end
      8'h14: begin n.t_lo = d[15:0]; n.t_hi = d[31:16]; end
      8'h15: begin n.i_min_r = d[15:0]; n.alpha_t = d[23:16]; end
      8'h16: begin n.tref = d[15:0]; n.ewma_shift = d[19:16]; n.drift_shift = d[23:20]; end
      8'h17: begin n.k_watch = d[7:0]; n.k_derate = d[15:8]; n.mad_floor = d[23:16]; n.hold_windows = d[31:24]; end
      8'h18: begin n.freeze_n = d[7:0]; n.forget_period = d[15:8]; n.k_novel = d[23:16]; n.arc_confirm = d[27:24]; end
      8'h19: begin n.hdc_qsh = d[15:0]; n.cls_qsh = d[31:16]; end
      8'h1A: begin n.e_gate = d[15:0]; n.hdc_drift_thr = d[31:16]; end
      8'h1B: begin n.ibin_shift = d[3:0]; n.tbin_thr = d[31:16]; end
      8'h1C: begin n.duty_normal = d[7:0]; n.duty_derate = d[15:8]; end
      default: ;
    endcase
    return n;
  endfunction

  logic is_cfg_addr;
  assign is_cfg_addr = (bus_addr >= 8'h10) && (bus_addr <= 8'h1C);

  logic m5b_ovr_s, m6_ovr_s;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      cfg          <= cfg_default();
      cfg_locked   <= 1'b0;
      trip_clear   <= 1'b0;
      base_reset   <= 1'b0;
      cls_reset    <= 1'b0;
      train_active <= 1'b0;
      train_label  <= '0;
      cfg_wr_err   <= 1'b0;
      cmd_rej      <= 1'b0;
      m5b_ovr_s    <= 1'b0;
      m6_ovr_s     <= 1'b0;
    end else begin
      trip_clear <= 1'b0;
      base_reset <= 1'b0;
      cls_reset  <= 1'b0;
      if (m5b_overrun) m5b_ovr_s <= 1'b1;
      if (m6_overrun)  m6_ovr_s  <= 1'b1;
      if (bus_wr) begin
        if (is_cfg_addr) begin
          if (cfg_locked) cfg_wr_err <= 1'b1;
          else            cfg <= cfg_write(cfg, bus_addr, bus_wdata);
        end else if (bus_addr == 8'h02) begin
          if (bus_wdata[0]) cfg_locked <= 1'b1;
          if (bus_wdata[1]) trip_clear <= 1'b1;
          if (bus_wdata[2] || bus_wdata[3] || bus_wdata[4]) begin
            if (contactor_close) begin
              cmd_rej <= 1'b1;
            end else begin
              if (bus_wdata[2]) base_reset <= 1'b1;
              if (bus_wdata[3]) cls_reset  <= 1'b1;
              if (bus_wdata[4]) begin
                train_active <= 1'b1;
                train_label  <= bus_wdata[9:8];
              end
            end
          end
          if (bus_wdata[5]) train_active <= 1'b0;
        end
      end
    end
  end

  // health index: 255 minus state penalty and HDC drift excess
  logic [15:0] excess;
  logic [16:0] pen;
  assign excess = (d_frozen > d_adapt) ? ((d_frozen - d_adapt) >> 1) : 16'd0;
  assign pen    = 17'(state) * 17'd48 + 17'(excess);
  assign health = (pen > 17'd255) ? 8'd0 : 8'(17'd255 - pen);

  // CP PWM
  localparam int PW = $clog2(PWM_PERIOD);
  logic [PW-1:0] pcnt;
  assign cp_duty = !contactor_close         ? 8'd0 :
                   (state == SEV_DERATE)    ? cfg.duty_derate : cfg.duty_normal;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      pcnt   <= '0;
      cp_pwm <= 1'b0;
    end else begin
      pcnt   <= (pcnt == PW'(PWM_PERIOD - 1)) ? '0 : pcnt + 1'b1;
      cp_pwm <= (32'(pcnt) << 8) < (32'(cp_duty) * 32'(PWM_PERIOD));
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      bus_rdata <= '0;
    end else if (bus_rd) begin
      unique case (bus_addr)
        8'h00: bus_rdata <= ID;
        8'h01: bus_rdata <= {6'd0, m6_ovr_s, m5b_ovr_s, cls_trained, m8_cause, cls,
                             train_active, m6_ready, m5b_ready, cmd_rej, cfg_wr_err,
                             contactor_close, cfg_locked, arc, stale, sensor_fault,
                             fsm_trip, hard_trip, state};
        8'h03: bus_rdata <= {th_idx, r_comp};
        8'h04: bus_rdata <= {8'd0, e_hf};
        8'h05: bus_rdata <= {d_frozen, d_adapt};
        8'h06: bus_rdata <= {16'd0, cp_duty, health};
        default: bus_rdata <= is_cfg_addr ? cfg_read(cfg, bus_addr) : 32'd0;
      endcase
    end
  end

endmodule
