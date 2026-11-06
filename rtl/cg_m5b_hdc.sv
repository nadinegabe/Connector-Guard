// M5b: per-charger HDC baseline with dual prototype.
//
// For each context bin the module keeps
//   * D signed saturating counters (CW bits)  -> P_adaptive = sign(counters)
//   * a D-bit frozen prototype P_frozen, captured once when the bin has seen
//     freeze_n learning updates
//   * EWMA mean / MAD of the normal Hamming distance to P_adaptive.
//
// Per window (f_valid with fok = 1):
//   SCORE : encode H chunk by chunk, d_a = |H ^ P_adaptive|, d_f = |H ^ P_frozen|
//   DECIDE: novelty (sudden)  4*d_a > 4*mean + k_novel * max(mad, DFLOOR)  -> DERATE
//           drift   (slow)    d_f - d_a > hdc_drift_thr                    -> WATCH
//   UPDATE: only if learn_ok and not novelty: C[d] += H[d] ? +1 : -1, with
//           counter halving every forget_period updates (forgetting)
//   FREEZE: only on the first time a bin reaches freeze_n updates.
// The frozen prototype is written only by FREEZE (once per bin) and by the
// clear pass that runs after reset and on base_reset.
//
// Throughput: about 3*D/64 + 4 cycles per window; a window arriving while
// busy is dropped and sets overrun.
module cg_m5b_hdc
  import cg_pkg::*;
#(
  parameter int D      = 1024,
  parameter int CW     = 8,
  // MAD floor in bits. One quantisation step of one feature changes about
  // STEP*3/8 ~ 13 bits of H at D = 1024, so the floor keeps single-level
  // jitter below the novelty threshold (k_novel/4 * DFLOOR = 48 bits).
  parameter int DFLOOR = 8
) (
  input  logic        clk,
  input  logic        rst_n,
  input  cfg_t        cfg,
  input  logic        f_valid,
  input  logic        fok,          // all features valid for this window
  input  logic [31:0] x [4],        // R_comp, th_idx, E_hf, ires_max
  input  logic [2:0]  bin,
  input  logic        learn_ok,
  input  logic        base_reset,
  output logic        ready,        // clear pass finished
  output logic        busy,
  output logic        done,
  output logic [1:0]  req,
  output logic        novelty,
  output logic        drift,
  output logic [15:0] d_adapt,
  output logic [15:0] d_frozen,
  output logic        learned,
  output logic        blocked,
  output logic        skipped,
  output logic        trained,
  output logic        froze,        // pulse: a frozen prototype was captured
  output logic        overrun
);
  localparam int C  = D / 64;
  localparam int CA = (C > 1) ? $clog2(C) : 1;
  localparam int MA = $clog2(NB * C);
  localparam logic signed [CW-1:0] CMAX = {1'b0, {(CW-1){1'b1}}};
  localparam logic signed [CW-1:0] CMIN = -CMAX;   // symmetric saturation

  typedef enum logic [2:0] {S_INIT, S_IDLE, S_SCORE, S_DECIDE, S_UPD, S_FRZ, S_DONE} st_t;
  st_t st;

  logic [64*CW-1:0] cnt_mem [NB*C];
  logic [63:0]      frz_mem [NB*C];
  logic [7:0]       upd_cnt [NB];
  logic             fz_v    [NB];
  logic [7:0]       fgt_cnt [NB];
  logic [23:0]      dmean   [NB];
  logic [23:0]      dmad    [NB];
  logic [7:0]       dcnt    [NB];

  logic [3:0]  lvl [4];
  logic [2:0]  bin_l;
  logic        learn_l;
  logic [CA-1:0] ci;
  logic [MA-1:0] init_a;
  logic [15:0] da, df;
  logic        nov_l, dr_l, do_learn, forget_l;

  logic [7:0] frz_n;
  assign frz_n = (cfg.freeze_n == 8'd0) ? 8'd1 : cfg.freeze_n;

  // encoder
  logic [63:0] hv;
  cg_hdc_enc #(.D(D), .SEED(64'h5EED_C0DE_0000_0001), .PA(0), .PB(2)) u_enc (
    .lvl(lvl), .chunk(16'(ci)), .hv(hv)
  );

  logic [MA-1:0]    addr;
  logic [64*CW-1:0] cnt_rd;
  logic [63:0]      frz_rd;
  logic [63:0]      pa;
  assign addr   = MA'({bin_l, ci});
  assign cnt_rd = cnt_mem[addr];
  assign frz_rd = frz_mem[addr];
  always_comb begin
    for (int k = 0; k < 64; k++) begin
      logic signed [CW-1:0] cv;
      cv = cnt_rd[k*CW +: CW];
      pa[k] = (cv > 0);
    end
  end

  // counter update lanes
  logic [64*CW-1:0] cnt_wr;
  always_comb begin
    for (int k = 0; k < 64; k++) begin
      logic signed [CW-1:0] cv;
      cv = cnt_rd[k*CW +: CW];
      if (forget_l) cv = cv >>> 1;
      if (hv[k]) cv = (cv == CMAX) ? cv : cv + 1'b1;
      else       cv = (cv == CMIN) ? cv : cv - 1'b1;
      cnt_wr[k*CW +: CW] = cv;
    end
  end

  // decision terms
  logic [15:0] mean_i, mad_i;
  logic [23:0] lhs, rhs;
  logic        nov_c, dr_c;
  assign mean_i = dmean[bin_l][23:8];
  assign mad_i  = (dmad[bin_l][23:8] > 16'(DFLOOR)) ? dmad[bin_l][23:8] : 16'(DFLOOR);
  assign lhs    = {6'd0, da, 2'b00};
  assign rhs    = {6'd0, mean_i, 2'b00} + 24'(mad_i) * 24'(cfg.k_novel);
  assign nov_c  = fz_v[bin_l] && (dcnt[bin_l] >= 8'd4) && (lhs > rhs);
  assign dr_c   = fz_v[bin_l] && (df > da) && ((df - da) > cfg.hdc_drift_thr);

  // distance statistics update (EWMA with warm-up)
  logic [3:0]         ssh;
  logic signed [25:0] e1, e2;
  logic [23:0]        ad;
  logic [23:0]        dmean_n, dmad_n;
  always_comb begin
    logic [7:0] c;
    c = dcnt[bin_l];
    if      (c < 8'd2)  ssh = 4'd1;
    else if (c < 8'd4)  ssh = 4'd2;
    else if (c < 8'd8)  ssh = 4'd3;
    else                ssh = 4'd4;
    e1 = 26'({da, 8'd0}) - 26'(dmean[bin_l]);
    ad = e1[25] ? 24'(-e1) : 24'(e1);
    e2 = 26'(ad) - 26'(dmad[bin_l]);
    if (c == 8'd0) begin
      dmean_n = {da, 8'd0};
      dmad_n  = 24'd0;
    end else begin
      dmean_n = 24'($signed({2'b00, dmean[bin_l]}) + (e1 >>> ssh));
      dmad_n  = 24'($signed({2'b00, dmad[bin_l]})  + (e2 >>> ssh));
    end
  end

  logic [7:0] upd_n;
  assign upd_n = (upd_cnt[bin_l] == 8'hFF) ? 8'hFF : upd_cnt[bin_l] + 8'd1;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st       <= S_INIT;
      init_a   <= '0;
      ready    <= 1'b0;
      busy     <= 1'b1;
      done     <= 1'b0;
      req      <= '0;
      novelty  <= 1'b0;
      drift    <= 1'b0;
      d_adapt  <= '0;
      d_frozen <= '0;
      learned  <= 1'b0;
      blocked  <= 1'b0;
      skipped  <= 1'b0;
      trained  <= 1'b0;
      froze    <= 1'b0;
      overrun  <= 1'b0;
      ci       <= '0;
      bin_l    <= '0;
      learn_l  <= 1'b0;
      da       <= '0;
      df       <= '0;
      nov_l    <= 1'b0;
      dr_l     <= 1'b0;
      do_learn <= 1'b0;
      forget_l <= 1'b0;
      for (int f = 0; f < 4; f++) lvl[f] <= '0;
      for (int b = 0; b < NB; b++) begin
        upd_cnt[b] <= '0; fz_v[b] <= 1'b0; fgt_cnt[b] <= '0;
        dmean[b] <= '0; dmad[b] <= '0; dcnt[b] <= '0;
      end
    end else begin
      done    <= 1'b0;
      learned <= 1'b0;
      blocked <= 1'b0;
      skipped <= 1'b0;
      froze   <= 1'b0;
      if (base_reset) begin
        st     <= S_INIT;
        init_a <= '0;
        ready  <= 1'b0;
        busy   <= 1'b1;
        for (int b = 0; b < NB; b++) begin
          upd_cnt[b] <= '0; fz_v[b] <= 1'b0; fgt_cnt[b] <= '0;
          dmean[b] <= '0; dmad[b] <= '0; dcnt[b] <= '0;
        end
      end else begin
        if (f_valid && st != S_IDLE) overrun <= 1'b1;
        unique case (st)
          S_INIT: begin
            cnt_mem[init_a] <= '0;
            frz_mem[init_a] <= '0;
            init_a <= init_a + 1'b1;
            if (init_a == MA'(NB * C - 1)) begin
              st    <= S_IDLE;
              ready <= 1'b1;
              busy  <= 1'b0;
            end
          end
          S_IDLE: begin
            if (f_valid) begin
              if (!fok) begin
                done    <= 1'b1;
                skipped <= 1'b1;
                req     <= SEV_NORMAL;
                novelty <= 1'b0;
                drift   <= 1'b0;
              end else begin
                for (int f = 0; f < 4; f++) lvl[f] <= quant(x[f], cfg.hdc_qsh[f*4 +: 4]);
                bin_l   <= bin;
                learn_l <= learn_ok;
                ci      <= '0;
                da      <= '0;
                df      <= '0;
                busy    <= 1'b1;
                st      <= S_SCORE;
              end
            end
          end
          S_SCORE: begin
            da <= da + 16'(popcount64(hv ^ pa));
            df <= df + 16'(popcount64(hv ^ frz_rd));
            if (ci == CA'(C - 1)) begin
              ci <= '0;
              st <= S_DECIDE;
            end else begin
              ci <= ci + 1'b1;
            end
          end
          S_DECIDE: begin
            nov_l    <= nov_c;
            dr_l     <= dr_c;
            do_learn <= learn_l && !nov_c;
            forget_l <= (cfg.forget_period != 8'd0) && (fgt_cnt[bin_l] + 8'd1 >= cfg.forget_period);
            if (learn_l && !nov_c) begin
              st <= S_UPD;
              if (upd_cnt[bin_l] >= (frz_n >> 1)) begin
                dmean[bin_l] <= dmean_n;
                dmad[bin_l]  <= dmad_n;
                dcnt[bin_l]  <= (dcnt[bin_l] == 8'hFF) ? 8'hFF : dcnt[bin_l] + 8'd1;
              end
            end else begin
              st <= S_DONE;
            end
          end
          S_UPD: begin
            cnt_mem[addr] <= cnt_wr;
            if (ci == CA'(C - 1)) begin
              ci <= '0;
              upd_cnt[bin_l] <= upd_n;
              fgt_cnt[bin_l] <= forget_l ? 8'd0 : fgt_cnt[bin_l] + 8'd1;
              st <= (!fz_v[bin_l] && upd_n >= frz_n) ? S_FRZ : S_DONE;
            end else begin
              ci <= ci + 1'b1;
            end
          end
          S_FRZ: begin
            frz_mem[addr] <= pa;
            if (ci == CA'(C - 1)) begin
              ci          <= '0;
              fz_v[bin_l] <= 1'b1;
              froze       <= 1'b1;
              st          <= S_DONE;
            end else begin
              ci <= ci + 1'b1;
            end
          end
          S_DONE: begin
            done     <= 1'b1;
            novelty  <= nov_l;
            drift    <= dr_l;
            req      <= nov_l ? SEV_DERATE : (dr_l ? SEV_WATCH : SEV_NORMAL);
            d_adapt  <= da;
            d_frozen <= df;
            learned  <= do_learn;
            blocked  <= !do_learn;
            trained  <= fz_v[bin_l];
            busy     <= 1'b0;
            st       <= S_IDLE;
          end
          default: st <= S_IDLE;
        endcase
      end
    end
  end

endmodule
