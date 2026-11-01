// M3: contact resistance estimate per window.
//   r_raw  = (max(v_avg,0) << 10) / i_avg        [1/1024 mOhm per LSB]
//   r_comp = r_raw - (r_raw * (t_avg - tref) * alpha_t) >>> 16, clamped >= 0
// The contact drop is a 4-wire (Kelvin) measurement, so the window ratio is
// used rather than a dV/dI slope; it needs no current step to be valid.
// Windows with i_avg < i_min_r produce done with r_valid = 0.
// Latency: 34 cycles from w_valid to done (divider 33 + output register).
module cg_m3_rcontact
  import cg_pkg::*;
(
  input  logic               clk,
  input  logic               rst_n,
  input  cfg_t               cfg,
  input  logic               w_valid,
  input  logic signed [15:0] i_avg,
  input  logic signed [15:0] v_avg,
  input  logic signed [15:0] t_avg,
  output logic               done,
  output logic               r_valid,
  output logic        [15:0] r_raw,
  output logic        [15:0] r_comp
);
  logic               div_start, div_busy, div_done, div0;
  logic [31:0]        div_num;
  logic [23:0]        div_den;
  logic [15:0]        div_q;
  logic signed [15:0] t_l;
  logic               low_i;

  assign low_i = (17'(i_avg) < $signed({1'b0, cfg.i_min_r})) || (i_avg <= 16'sd0);

  always_comb begin
    div_start = w_valid && !low_i;
    div_num   = (v_avg > 16'sd0) ? (32'(v_avg) << 10) : 32'd0;
    div_den   = 24'(i_avg);
  end

  cg_div #(.NW(32), .DW(24), .QW(16)) u_div (
    .clk, .rst_n,
    .start(div_start), .num(div_num), .den(div_den),
    .busy(div_busy), .done(div_done), .quot(div_q), .div0(div0)
  );

  logic signed [16:0] dt;
  logic signed [42:0] prod;
  logic signed [42:0] corr;
  logic signed [42:0] rc;
  assign dt   = 17'(t_l) - 17'(cfg.tref);
  assign prod = 43'($signed({1'b0, div_q})) * 43'(dt) * 43'($signed({1'b0, cfg.alpha_t}));
  assign corr = prod >>> 16;
  assign rc   = 43'($signed({1'b0, div_q})) - corr;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      done    <= 1'b0;
      r_valid <= 1'b0;
      r_raw   <= '0;
      r_comp  <= '0;
      t_l     <= '0;
    end else begin
      done <= 1'b0;
      if (w_valid) begin
        t_l <= t_avg;
        if (low_i) begin
          done    <= 1'b1;
          r_valid <= 1'b0;
        end
      end
      if (div_done) begin
        done    <= 1'b1;
        r_valid <= !div0;
        r_raw   <= div_q;
        r_comp  <= (rc < 43'sd0) ? 16'd0 : (rc > 43'sd65535) ? 16'hFFFF : rc[15:0];
      end
    end
  end

  logic unused;
  assign unused = div_busy;

endmodule
