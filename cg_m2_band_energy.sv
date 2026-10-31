// M2: HF band features of the AC current.
// Direct-form-I biquad band-pass (Q2.14 coefficients; defaults are an RBJ
// band-pass at 30 kHz, Q = 0.5 for fs = 250 kS/s, i.e. roughly 5-100 kHz),
// followed by per-window features:
//   e    : mean(y^2) >> ESH          (band energy, 24-bit saturating)
//   peak : max |y|
//   zc   : sign changes of y with +/-ZC_HYST hysteresis
//   de   : |e - e of previous window| (energy instability)
module cg_m2_band_energy
  import cg_pkg::*;
#(
  parameter logic signed [15:0] B0 = 16'sd6658,
  parameter logic signed [15:0] B1 = 16'sd0,
  parameter logic signed [15:0] B2 = -16'sd6658,
  parameter logic signed [15:0] A1 = -16'sd14179,
  parameter logic signed [15:0] A2 = 16'sd3069,
  parameter int ESH      = 4,
  parameter int ZC_HYST  = 8
) (
  input  logic               clk,
  input  logic               rst_n,
  input  cfg_t               cfg,
  input  logic               ac_valid,
  input  logic signed [15:0] ac,
  input  logic               ac_last,
  // filtered sample (observability)
  output logic               y_valid,
  output logic signed [15:0] y,
  // window features
  output logic               f_valid,
  output logic        [23:0] e,
  output logic        [15:0] peak,
  output logic        [9:0]  zc,
  output logic        [23:0] de
);
  logic [3:0] wl;
  assign wl = (cfg.win_log2 < 4'd3) ? 4'd3 : ((cfg.win_log2 > 4'd10) ? 4'd10 : cfg.win_log2);

  // ---------------- stage 1: biquad ----------------
  logic signed [15:0] x1, x2, y1, y2;
  logic signed [34:0] acc;
  logic signed [20:0] ysh;
  logic signed [15:0] ynew;
  logic               y_last;

  assign acc = (35'(B0) * 35'(ac)) + (35'(B1) * 35'(x1)) + (35'(B2) * 35'(x2))
             - (35'(A1) * 35'(y1)) - (35'(A2) * 35'(y2));
  assign ysh = 21'(acc >>> 14);
  assign ynew = (ysh > 21'sd32767) ? 16'sh7FFF : (ysh < -21'sd32768) ? -16'sh8000 : ysh[15:0];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      x1 <= '0; x2 <= '0; y1 <= '0; y2 <= '0;
      y_valid <= 1'b0;
      y       <= '0;
      y_last  <= 1'b0;
    end else begin
      y_valid <= ac_valid;
      y_last  <= ac_valid && ac_last;
      if (ac_valid) begin
        x2 <= x1; x1 <= ac;
        y2 <= y1; y1 <= ynew;
        y  <= ynew;
      end
    end
  end

  // ---------------- stage 2: window features ----------------
  logic [47:0] acc_e;
  logic [15:0] pk;
  logic [9:0]  zcnt;
  logic [1:0]  sgn;        // 0 = none yet, 1 = positive, 2 = negative
  logic [23:0] e_prev;

  logic [15:0] ya;
  logic [31:0] y2u;
  logic [47:0] acc_n;
  logic [15:0] pk_n;
  logic [1:0]  sgn_n;
  logic        flip;
  logic [47:0] e_full;
  logic [23:0] e_n;

  assign ya    = y[15] ? 16'(-y) : y;
  assign y2u   = 32'(ya) * 32'(ya);
  assign acc_n = acc_e + 48'(y2u);
  assign pk_n  = (ya > pk) ? ya : pk;
  always_comb begin
    sgn_n = sgn;
    if (y > 16'(ZC_HYST))       sgn_n = 2'd1;
    else if (y < -16'(ZC_HYST)) sgn_n = 2'd2;
  end
  assign flip   = (sgn != 2'd0) && (sgn_n != sgn);
  assign e_full = acc_n >> (wl + 4'(ESH));
  assign e_n    = (e_full > 48'hFFFFFF) ? 24'hFFFFFF : e_full[23:0];

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      acc_e   <= '0;
      pk      <= '0;
      zcnt    <= '0;
      sgn     <= '0;
      e_prev  <= '0;
      f_valid <= 1'b0;
      e       <= '0;
      peak    <= '0;
      zc      <= '0;
      de      <= '0;
    end else begin
      f_valid <= 1'b0;
      if (y_valid) begin
        sgn <= sgn_n;
        if (y_last) begin
          f_valid <= 1'b1;
          e       <= e_n;
          peak    <= pk_n;
          zc      <= (zcnt == 10'h3FF) ? zcnt : zcnt + 10'(flip);
          de      <= (e_n > e_prev) ? e_n - e_prev : e_prev - e_n;
          e_prev  <= e_n;
          acc_e   <= '0;
          pk      <= '0;
          zcnt    <= '0;
        end else begin
          acc_e <= acc_n;
          pk    <= pk_n;
          if (flip && zcnt != 10'h3FF) zcnt <= zcnt + 10'd1;
        end
      end
    end
  end

endmodule
