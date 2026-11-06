// ConnectorGuard-HDC top level.
//
//  M0 input/replay -> M1 preproc -> M2 band energy ----------------> M6 HDC classifier --+
//                              \-> M3 R contact  \                                         |
//                              \-> M4 thermal     >-- collector -> M5a EWMA ---------------+-> M7 FSM -> M9
//                                                 /             -> M5b HDC baseline -------+
//  M0 raw samples -> M8 hard trip ------------------------------------------------------------> M9 contactor gate
//
// M8 sees only the raw M0 sample stream and its trip goes to the contactor
// gate in M9 combinationally. The AI path (M5a/M5b/M6 -> M7) can only add
// reasons to open the contactor.
module cg_top
  import cg_pkg::*;
#(
  parameter int RP_DEPTH   = 1024,
  parameter int D_BASE     = 1024,
  parameter int D_CLS      = 512,
  parameter int PWM_PERIOD = 1000
) (
  input  logic        clk,
  input  logic        rst_n,
  // live ADC stream
  input  logic        live_valid,
  input  sample_t     live_data,
  output logic        live_ready,
  // replay (test infrastructure)
  input  logic        src_sel,
  input  logic        rp_wr_en,
  input  logic [$clog2(RP_DEPTH)-1:0] rp_wr_addr,
  input  sample_t     rp_wr_data,
  input  logic        rp_start,
  input  logic [$clog2(RP_DEPTH):0]   rp_len,
  input  logic [15:0] rp_period,
  output logic        rp_busy,
  // register bus
  input  logic        bus_wr,
  input  logic        bus_rd,
  input  logic [7:0]  bus_addr,
  input  logic [31:0] bus_wdata,
  output logic [31:0] bus_rdata,
  // outputs
  output logic        contactor_close,
  output logic        cp_pwm,
  output logic [7:0]  cp_duty,
  output logic [1:0]  state,
  output logic        hard_trip,
  output logic [7:0]  health
);
  cfg_t cfg;
  logic cfg_locked, trip_clear, base_reset, cls_reset, train_active;
  logic [1:0] train_label;
  logic cfg_wr_err, cmd_rej;

  // ---------------- M0 ----------------
  logic    s_valid, s_win_last, stale;
  sample_t s_data;
  cg_m0_input #(.RP_DEPTH(RP_DEPTH)) u_m0 (
    .clk, .rst_n, .cfg,
    .live_valid, .live_data, .live_ready,
    .src_sel, .rp_wr_en, .rp_wr_addr, .rp_wr_data, .rp_start, .rp_len, .rp_period, .rp_busy,
    .s_valid, .s_data, .s_win_last, .stale
  );

  // ---------------- M8 (independent) ----------------
  logic       m8_latched, m8_armed;
  logic [4:0] m8_cause;
  cg_m8_hardtrip u_m8 (
    .clk, .rst_n, .cfg,
    .s_valid, .s_data, .trip_clear,
    .hard_trip, .trip_latched(m8_latched), .cause(m8_cause), .armed(m8_armed)
  );

  // ---------------- M1 ----------------
  logic               ac_valid, ac_last, w_valid, w_fault, sensor_fault;
  logic signed [15:0] ac, i_avg, v_avg, t_avg, tamb_avg;
  logic        [15:0] ires_max;
  cg_m1_preproc u_m1 (
    .clk, .rst_n, .cfg,
    .s_valid, .s_data, .s_win_last,
    .ac_valid, .ac, .ac_last,
    .w_valid, .i_avg, .v_avg, .t_avg, .tamb_avg, .ires_max, .w_fault, .sensor_fault
  );

  // ---------------- M2 ----------------
  logic               y_valid, m2_valid;
  logic signed [15:0] y;
  logic [23:0]        e_hf, de_hf;
  logic [15:0]        peak_hf;
  logic [9:0]         zc_hf;
  cg_m2_band_energy u_m2 (
    .clk, .rst_n, .cfg,
    .ac_valid, .ac, .ac_last,
    .y_valid, .y,
    .f_valid(m2_valid), .e(e_hf), .peak(peak_hf), .zc(zc_hf), .de(de_hf)
  );

  // ---------------- M3 / M4 ----------------
  logic        m3_done, r_valid;
  logic [15:0] r_raw, r_comp;
  cg_m3_rcontact u_m3 (
    .clk, .rst_n, .cfg,
    .w_valid, .i_avg, .v_avg, .t_avg,
    .done(m3_done), .r_valid, .r_raw, .r_comp
  );

  logic               m4_done, th_valid;
  logic [15:0]        th_idx;
  logic signed [15:0] t_rate;
  cg_m4_thermal u_m4 (
    .clk, .rst_n, .cfg,
    .w_valid, .i_avg, .t_avg, .tamb_avg,
    .done(m4_done), .th_valid, .th_idx, .t_rate
  );

  // ---------------- feature collector ----------------
  logic        got3, got4, feat_valid;
  logic [2:0]  bin_l;
  logic [15:0] ires_l;
  logic [23:0] e_l;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      got3       <= 1'b0;
      got4       <= 1'b0;
      feat_valid <= 1'b0;
      bin_l      <= '0;
      ires_l     <= '0;
      e_l        <= '0;
    end else begin
      feat_valid <= 1'b0;
      if (w_valid) begin
        bin_l  <= ctx_bin(i_avg, tamb_avg, cfg.ibin_shift, cfg.tbin_thr);
        ires_l <= ires_max;
        got3   <= 1'b0;
        got4   <= 1'b0;
      end
      if (m2_valid) e_l <= e_hf;
      if (m3_done) got3 <= 1'b1;
      if (m4_done) got4 <= 1'b1;
      if ((got3 || m3_done) && (got4 || m4_done) && !w_valid) begin
        feat_valid <= 1'b1;
        got3       <= 1'b0;
        got4       <= 1'b0;
      end
    end
  end

  // learning permission (shared by M5a and M5b)
  logic m7_learn_ok, arc, learn_ok;
  assign learn_ok = m7_learn_ok && cfg.learn_en && cfg_locked && !sensor_fault &&
                    !stale && !hard_trip && !arc;

  // ---------------- M5a ----------------
  logic        m5a_done, m5a_learned, m5a_blocked;
  logic [1:0]  m5a_req, m5a_anom, m5a_drift, m5a_trained;
  logic [15:0] m5a_x [2];
  logic [15:0] m5a_mean [2];
  logic [15:0] m5a_frz [2];
  assign m5a_x[0] = r_comp;
  assign m5a_x[1] = th_idx;
  cg_m5a_ewma #(.NFE(2)) u_m5a (
    .clk, .rst_n, .cfg,
    .f_valid(feat_valid), .x(m5a_x), .xv({th_valid, r_valid}), .bin(bin_l),
    .learn_ok, .base_reset,
    .done(m5a_done), .req(m5a_req), .anom(m5a_anom), .drift(m5a_drift),
    .learned(m5a_learned), .blocked(m5a_blocked), .trained(m5a_trained),
    .mean_out(m5a_mean), .frozen_out(m5a_frz)
  );

  // ---------------- M5b ----------------
  logic        m5b_ready, m5b_busy, m5b_done, m5b_nov, m5b_drift;
  logic        m5b_learned, m5b_blocked, m5b_skipped, m5b_trained, m5b_froze, m5b_ovr;
  logic [1:0]  m5b_req;
  logic [15:0] d_adapt, d_frozen;
  logic [31:0] m5b_x [4];
  assign m5b_x[0] = 32'(r_comp);
  assign m5b_x[1] = 32'(th_idx);
  assign m5b_x[2] = 32'(e_l);
  assign m5b_x[3] = 32'(ires_l);
  cg_m5b_hdc #(.D(D_BASE)) u_m5b (
    .clk, .rst_n, .cfg,
    .f_valid(feat_valid), .fok(r_valid && th_valid), .x(m5b_x), .bin(bin_l),
    .learn_ok, .base_reset,
    .ready(m5b_ready), .busy(m5b_busy), .done(m5b_done), .req(m5b_req),
    .novelty(m5b_nov), .drift(m5b_drift), .d_adapt, .d_frozen,
    .learned(m5b_learned), .blocked(m5b_blocked), .skipped(m5b_skipped),
    .trained(m5b_trained), .froze(m5b_froze), .overrun(m5b_ovr)
  );

  // ---------------- M6 ----------------
  logic            m6_ready, m6_done, m6_trained_win, m6_ovr;
  logic [1:0]      cls;
  logic [NCLS-1:0] cls_trained;
  logic [15:0]     cls_dist [NCLS];
  cg_m6_hdc_cls #(.D(D_CLS)) u_m6 (
    .clk, .rst_n, .cfg,
    .f_valid(m2_valid), .e(e_hf), .peak(peak_hf), .zc(zc_hf), .de(de_hf),
    .train_active, .train_label, .train_en(!contactor_close), .cls_reset,
    .ready(m6_ready), .done(m6_done), .cls, .arc, .trained_win(m6_trained_win),
    .cls_trained, .cdist(cls_dist), .overrun(m6_ovr)
  );

  // ---------------- M7 ----------------
  // M7 sees TRIP_CLEAR one cycle after M8, so a clear that releases M8 can
  // also release the FSM in the same command.
  logic       trip_clear_d;
  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) trip_clear_d <= 1'b0;
    else        trip_clear_d <= trip_clear;
  end

  logic [1:0] req_now;
  logic       fsm_trip, clear_rejected;
  cg_m7_fsm u_m7 (
    .clk, .rst_n, .cfg,
    .a_valid(m5a_done), .a_req(m5a_req),
    .b_valid(m5b_done), .b_req(m5b_req),
    .c_valid(m6_done),  .c_arc(arc),
    .sensor_fault, .stale, .hard_trip(m8_latched), .trip_clear(trip_clear_d),
    .state, .req_now, .fsm_trip, .learn_ok(m7_learn_ok), .clear_rejected
  );

  // ---------------- M9 ----------------
  cg_m9_regs #(.PWM_PERIOD(PWM_PERIOD)) u_m9 (
    .clk, .rst_n,
    .bus_wr, .bus_rd, .bus_addr, .bus_wdata, .bus_rdata,
    .cfg, .cfg_locked, .trip_clear, .base_reset, .cls_reset, .train_active, .train_label,
    .cfg_wr_err, .cmd_rej,
    .state, .hard_trip, .m8_cause, .fsm_trip, .sensor_fault, .stale, .arc, .cls,
    .cls_trained, .m5b_ready, .m6_ready, .m5b_overrun(m5b_ovr), .m6_overrun(m6_ovr),
    .r_comp, .th_idx, .e_hf, .d_adapt, .d_frozen,
    .contactor_close, .cp_duty, .cp_pwm, .health
  );

endmodule
