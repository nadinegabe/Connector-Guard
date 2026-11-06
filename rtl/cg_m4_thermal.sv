// M4: thermal index and temperature rate per window.
//   th_idx = (max(t_avg - tamb_avg, 0) << 20) / ((i_avg^2) >> 10)
//            (temperature rise per I^2; rises when contact loss grows)
//   t_rate = t_avg - t_avg of previous window (signed, saturating)
// Windows with i_avg < i_min_r produce done with th_valid = 0.
// Latency: 42 cycles from w_valid to done (divider 41 + output register).
module cg_m4_thermal
  import cg_pkg::*;
(
  input  logic               clk,
  input  logic               rst_n,
  input  cfg_t               cfg,
  input  logic               w_valid,
  input  logic signed [15:0] i_avg,
  input  logic signed [15:0] t_avg,
  input  logic signed [15:0] tamb_avg,
  output logic               done,
  output logic               th_valid,
  output logic        [15:0] th_idx,
  output logic signed [15:0] t_rate
);
  logic               div_start, div_busy, div_done, div0;
  logic [39:0]        div_num;
  logic [23:0]        div_den;
  logic [15:0]        div_q;
  logic               low_i;
  logic signed [15:0] t_prev;
  logic               have_prev;

  logic signed [16:0] dtemp;
  logic        [31:0] i2;
  assign dtemp = 17'(t_avg) - 17'(tamb_avg);
  assign i2    = 32'($unsigned(i_avg)) * 32'($unsigned(i_avg));
  assign low_i = (17'(i_avg) < $signed({1'b0, cfg.i_min_r})) || (i_avg <= 16'sd0);

  always_comb begin
    div_start = w_valid && !low_i;
    div_num   = (dtemp > 17'sd0) ? (40'(dtemp) << 20) : 40'd0;
    div_den   = 24'(i2 >> 10);
  end

  cg_div #(.NW(40), .DW(24), .QW(16)) u_div (
    .clk, .rst_n,
    .start(div_start), .num(div_num), .den(div_den),
    .busy(div_busy), .done(div_done), .quot(div_q), .div0(div0)
  );

  logic signed [16:0] rate_w;
  assign rate_w = 17'(t_avg) - 17'(t_prev);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      done      <= 1'b0;
      th_valid  <= 1'b0;
      th_idx    <= '0;
      t_rate    <= '0;
      t_prev    <= '0;
      have_prev <= 1'b0;
    end else begin
      done <= 1'b0;
      if (w_valid) begin
        t_prev    <= t_avg;
        have_prev <= 1'b1;
        t_rate    <= !have_prev ? 16'sd0 :
                     (rate_w > 17'sd32767)  ? 16'sh7FFF :
                     (rate_w < -17'sd32768) ? -16'sh8000 : rate_w[15:0];
        if (low_i) begin
          done     <= 1'b1;
          th_valid <= 1'b0;
        end
      end
      if (div_done) begin
        done     <= 1'b1;
        th_valid <= !div0;
        th_idx   <= div_q;
      end
    end
  end

  logic unused;
  assign unused = div_busy;

endmodule
