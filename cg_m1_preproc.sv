// M1: pre-processing and sensor validation.
//  * splits load current into DC (EWMA, alpha = 1/16) and AC (i - DC) parts;
//    the AC part feeds M2 with one cycle of latency
//  * window averages of i, v, t, tamb and window maximum of ires
//  * plausibility: NTC / ambient outside [t_lo, t_hi] or ADC rail hits on
//    i or v raise sensor_fault. The fault is set on the offending sample and
//    cleared only at the end of a window with no offending samples.
module cg_m1_preproc
  import cg_pkg::*;
(
  input  logic        clk,
  input  logic        rst_n,
  input  cfg_t        cfg,
  input  logic        s_valid,
  input  sample_t     s_data,
  input  logic        s_win_last,
  // AC stream to M2
  output logic               ac_valid,
  output logic signed [15:0] ac,
  output logic               ac_last,
  // window outputs
  output logic               w_valid,
  output logic signed [15:0] i_avg,
  output logic signed [15:0] v_avg,
  output logic signed [15:0] t_avg,
  output logic signed [15:0] tamb_avg,
  output logic        [15:0] ires_max,
  output logic               w_fault,
  output logic               sensor_fault
);
  logic [3:0] wl;
  assign wl = (cfg.win_log2 < 4'd3) ? 4'd3 : ((cfg.win_log2 > 4'd10) ? 4'd10 : cfg.win_log2);

  logic signed [25:0] sum_i, sum_v, sum_t, sum_ta;
  logic        [15:0] mx_ires;
  logic               bad_win;
  logic signed [21:0] idc_q;   // DC estimate, 4 fractional bits
  logic               idc_init;

  logic bad_now;
  assign bad_now = (s_data.t < cfg.t_lo) || (s_data.t > cfg.t_hi) ||
                   (s_data.tamb < cfg.t_lo) || (s_data.tamb > cfg.t_hi) ||
                   (s_data.i == 16'sh7FFF) || (s_data.i == -16'sh8000) ||
                   (s_data.v == 16'sh7FFF) || (s_data.v == -16'sh8000);

  logic signed [21:0] i_q;
  logic signed [21:0] idc_next;
  logic signed [17:0] ac_wide;
  assign i_q      = 22'(s_data.i) <<< 4;
  assign idc_next = idc_init ? (idc_q + ((i_q - idc_q) >>> 4)) : i_q;
  assign ac_wide  = 18'(s_data.i) - 18'(idc_q >>> 4);

  // next-sum values including the current sample
  logic signed [25:0] nsum_i, nsum_v, nsum_t, nsum_ta;
  logic        [15:0] nmx;
  assign nsum_i  = sum_i  + 26'(s_data.i);
  assign nsum_v  = sum_v  + 26'(s_data.v);
  assign nsum_t  = sum_t  + 26'(s_data.t);
  assign nsum_ta = sum_ta + 26'(s_data.tamb);
  assign nmx     = (s_data.ires > mx_ires) ? s_data.ires : mx_ires;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      ac_valid     <= 1'b0;
      ac           <= '0;
      ac_last      <= 1'b0;
      w_valid      <= 1'b0;
      i_avg        <= '0;
      v_avg        <= '0;
      t_avg        <= '0;
      tamb_avg     <= '0;
      ires_max     <= '0;
      w_fault      <= 1'b0;
      sensor_fault <= 1'b0;
      sum_i        <= '0;
      sum_v        <= '0;
      sum_t        <= '0;
      sum_ta       <= '0;
      mx_ires      <= '0;
      bad_win      <= 1'b0;
      idc_q        <= '0;
      idc_init     <= 1'b0;
    end else begin
      ac_valid <= s_valid;
      ac_last  <= s_valid && s_win_last;
      w_valid  <= 1'b0;
      if (s_valid) begin
        idc_q    <= idc_next;
        idc_init <= 1'b1;
        ac       <= (!idc_init) ? 16'sd0 :
                    (ac_wide > 18'sd32767)  ? 16'sh7FFF :
                    (ac_wide < -18'sd32768) ? -16'sh8000 : ac_wide[15:0];
        if (bad_now) sensor_fault <= 1'b1;
        if (s_win_last) begin
          w_valid  <= 1'b1;
          i_avg    <= 16'(nsum_i  >>> wl);
          v_avg    <= 16'(nsum_v  >>> wl);
          t_avg    <= 16'(nsum_t  >>> wl);
          tamb_avg <= 16'(nsum_ta >>> wl);
          ires_max <= nmx;
          w_fault  <= bad_win || bad_now;
          if (!(bad_win || bad_now)) sensor_fault <= 1'b0;
          sum_i   <= '0;
          sum_v   <= '0;
          sum_t   <= '0;
          sum_ta  <= '0;
          mx_ires <= '0;
          bad_win <= 1'b0;
        end else begin
          sum_i   <= nsum_i;
          sum_v   <= nsum_v;
          sum_t   <= nsum_t;
          sum_ta  <= nsum_ta;
          mx_ires <= nmx;
          bad_win <= bad_win || bad_now;
        end
      end
    end
  end

endmodule
