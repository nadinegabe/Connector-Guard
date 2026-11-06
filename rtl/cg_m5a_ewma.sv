// M5a: per-bin EWMA baseline for scalar drift (R_comp, thermal index).
// Per feature and context bin it keeps an adaptive mean and mean absolute
// deviation (Q16.8), a learning-update counter, and a frozen mean captured
// once when the counter reaches freeze_n.
//
// Severity request per window (only for trained bins, positive deviation):
//   WATCH  if 4*(x - mean) > k_watch  * max(mad, mad_floor)
//   DERATE if 4*(x - mean) > k_derate * max(mad, mad_floor)
//   drift: WATCH  if mean - frozen > frozen >> drift_shift
//          DERATE if mean - frozen > frozen >> (drift_shift - 1)
// Learning happens only when learn_ok is high, the feature is valid and the
// window is not itself a deviation anomaly. Warm-up uses a shorter EWMA
// shift (running average) until the configured shift is reached.
// The frozen mean changes only on its first capture or on base_reset.
module cg_m5a_ewma
  import cg_pkg::*;
#(
  parameter int NFE = 2
) (
  input  logic              clk,
  input  logic              rst_n,
  input  cfg_t              cfg,
  input  logic              f_valid,
  input  logic [15:0]       x  [NFE],
  input  logic [NFE-1:0]    xv,
  input  logic [2:0]        bin,
  input  logic              learn_ok,
  input  logic              base_reset,
  output logic              done,
  output logic [1:0]        req,
  output logic [NFE-1:0]    anom,
  output logic [NFE-1:0]    drift,
  output logic              learned,
  output logic              blocked,
  output logic [NFE-1:0]    trained,
  output logic [15:0]       mean_out  [NFE],
  output logic [15:0]       frozen_out[NFE]
);
  logic [23:0] mean_m [NFE][NB];
  logic [23:0] mad_m  [NFE][NB];
  logic [23:0] frz_m  [NFE][NB];
  logic [7:0]  cnt_m  [NFE][NB];
  logic        fv_m   [NFE][NB];

  logic [7:0] frz_n;
  assign frz_n = (cfg.freeze_n == 8'd0) ? 8'd1 : cfg.freeze_n;

  // per-feature combinational evaluation
  logic [1:0]  req_f   [NFE];
  logic        anom_f  [NFE];
  logic        drift_f [NFE];
  logic        upd_f   [NFE];
  logic [23:0] mean_n  [NFE];
  logic [23:0] mad_n   [NFE];
  logic [7:0]  cnt_n   [NFE];
  logic        frz_now [NFE];

  always_comb begin
    for (int f = 0; f < NFE; f++) begin
      logic [23:0] m, md, fz, xq;
      logic [7:0]  c;
      logic        fv;
      logic [15:0] mint, madf, devp;
      logic [17:0] dev4;
      logic [23:0] thw, thd;
      logic [3:0]  sh;
      logic signed [25:0] dq, dmad;
      logic [23:0] adq;
      logic [1:0]  dl;
      logic [23:0] dd, lim1, lim2;
      logic [3:0]  ds1;

      m    = mean_m[f][bin];
      md   = mad_m[f][bin];
      fz   = frz_m[f][bin];
      c    = cnt_m[f][bin];
      fv   = fv_m[f][bin];
      xq   = {x[f], 8'd0};
      mint = m[23:8];
      madf = (md[23:8] > {8'd0, cfg.mad_floor}) ? md[23:8] : {8'd0, cfg.mad_floor};
      devp = (x[f] > mint) ? (x[f] - mint) : 16'd0;
      dev4 = {devp, 2'b00};
      thw  = 24'(madf) * 24'(cfg.k_watch);
      thd  = 24'(madf) * 24'(cfg.k_derate);

      dl = 2'd0;
      if (fv && 24'(dev4) > thd)      dl = 2'd2;
      else if (fv && 24'(dev4) > thw) dl = 2'd1;
      anom_f[f] = xv[f] && (dl != 2'd0);

      // drift of adaptive mean against frozen mean
      ds1  = (cfg.drift_shift == 4'd0) ? 4'd0 : cfg.drift_shift - 4'd1;
      dd   = (m > fz) ? (m - fz) : 24'd0;
      lim1 = fz >> cfg.drift_shift;
      lim2 = fz >> ds1;
      drift_f[f] = xv[f] && fv && (dd > lim1);
      if (xv[f] && fv && cfg.drift_shift != 4'd0 && dd > lim2) dl = 2'd2;
      else if (drift_f[f] && dl == 2'd0)                         dl = 2'd1;
      req_f[f] = xv[f] ? dl : 2'd0;

      // update values
      if      (c < 8'd2)  sh = 4'd1;
      else if (c < 8'd4)  sh = 4'd2;
      else if (c < 8'd8)  sh = 4'd3;
      else if (c < 8'd16) sh = 4'd4;
      else if (c < 8'd32) sh = 4'd5;
      else if (c < 8'd64) sh = 4'd6;
      else                sh = 4'd7;
      if (sh > cfg.ewma_shift) sh = (cfg.ewma_shift == 4'd0) ? 4'd1 : cfg.ewma_shift;

      dq   = 26'(xq) - 26'(m);
      adq  = dq[25] ? 24'(-dq) : 24'(dq);
      dmad = 26'(adq) - 26'(md);
      if (c == 8'd0) begin
        mean_n[f] = xq;
        mad_n[f]  = 24'd0;
      end else begin
        mean_n[f] = 24'($signed({2'b00, m})  + (dq   >>> sh));
        mad_n[f]  = 24'($signed({2'b00, md}) + (dmad >>> sh));
      end
      cnt_n[f]   = (c == 8'hFF) ? c : c + 8'd1;
      upd_f[f]   = learn_ok && xv[f] && !anom_f[f];
      frz_now[f] = upd_f[f] && !fv && (cnt_n[f] >= frz_n);
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      for (int f = 0; f < NFE; f++)
        for (int b = 0; b < NB; b++) begin
          mean_m[f][b] <= '0; mad_m[f][b] <= '0; frz_m[f][b] <= '0;
          cnt_m[f][b] <= '0;  fv_m[f][b] <= 1'b0;
        end
      done    <= 1'b0;
      req     <= '0;
      anom    <= '0;
      drift   <= '0;
      learned <= 1'b0;
      blocked <= 1'b0;
      trained <= '0;
      for (int f = 0; f < NFE; f++) begin
        mean_out[f]   <= '0;
        frozen_out[f] <= '0;
      end
    end else if (base_reset) begin
      for (int f = 0; f < NFE; f++)
        for (int b = 0; b < NB; b++) begin
          mean_m[f][b] <= '0; mad_m[f][b] <= '0; frz_m[f][b] <= '0;
          cnt_m[f][b] <= '0;  fv_m[f][b] <= 1'b0;
        end
      done    <= 1'b0;
      req     <= '0;
      anom    <= '0;
      drift   <= '0;
      learned <= 1'b0;
      blocked <= 1'b0;
      trained <= '0;
    end else begin
      done    <= f_valid;
      learned <= 1'b0;
      blocked <= 1'b0;
      if (f_valid) begin
        logic [1:0] r;
        logic       any_upd, any_valid;
        r = 2'd0;
        any_upd = 1'b0;
        any_valid = 1'b0;
        for (int f = 0; f < NFE; f++) begin
          r = sev_max(r, req_f[f]);
          anom[f]    <= anom_f[f];
          drift[f]   <= drift_f[f];
          trained[f] <= fv_m[f][bin] || frz_now[f];
          any_valid  = any_valid || xv[f];
          if (upd_f[f]) begin
            any_upd = 1'b1;
            mean_m[f][bin] <= mean_n[f];
            mad_m[f][bin]  <= mad_n[f];
            cnt_m[f][bin]  <= cnt_n[f];
            if (frz_now[f]) begin
              frz_m[f][bin] <= mean_n[f];
              fv_m[f][bin]  <= 1'b1;
            end
          end
          mean_out[f]   <= upd_f[f] ? mean_n[f][23:8] : mean_m[f][bin][23:8];
          frozen_out[f] <= frz_now[f] ? mean_n[f][23:8] : frz_m[f][bin][23:8];
        end
        req     <= r;
        learned <= any_upd;
        blocked <= any_valid && !any_upd;
      end
    end
  end

endmodule
