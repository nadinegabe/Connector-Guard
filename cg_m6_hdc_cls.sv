// M6: HDC classifier for HF noise: background / switching / arc.
//
// Features per window from M2: band energy e, peak, zero crossings, |dE|.
// Windows with e < e_gate are BACKGROUND without running the HDC.
// Class prototypes are counter arrays (sign = prototype) trained on chip:
// while train_active is high AND training is permitted (train_en, i.e. the
// contactor is open) every window is added to class train_label. Training
// windows never raise arc. Inference picks the trained class with the
// smallest Hamming distance (ties go to the lower class index).
// arc is raised when arc_confirm consecutive windows classify as ARC.
//
// Latency: D/64 + 3 cycles from f_valid to done.
module cg_m6_hdc_cls
  import cg_pkg::*;
#(
  parameter int D  = 512,
  parameter int CW = 8
) (
  input  logic        clk,
  input  logic        rst_n,
  input  cfg_t        cfg,
  input  logic        f_valid,
  input  logic [23:0] e,
  input  logic [15:0] peak,
  input  logic [9:0]  zc,
  input  logic [23:0] de,
  input  logic        train_active,
  input  logic [1:0]  train_label,
  input  logic        train_en,
  input  logic        cls_reset,
  output logic        ready,
  output logic        done,
  output logic [1:0]  cls,
  output logic        arc,
  output logic        trained_win,   // pulse: a window was used for training
  output logic [NCLS-1:0] cls_trained,
  output logic [15:0] cdist [NCLS],
  output logic        overrun
);
  localparam int C  = D / 64;
  localparam int CA = (C > 1) ? $clog2(C) : 1;
  localparam logic signed [CW-1:0] CMAX = {1'b0, {(CW-1){1'b1}}};
  localparam logic signed [CW-1:0] CMIN = -CMAX;   // symmetric saturation

  typedef enum logic [2:0] {S_INIT, S_IDLE, S_SCORE, S_TRAIN, S_DONE} st_t;
  st_t st;

  logic [64*CW-1:0] mem [NCLS][C];
  logic [15:0]      ncnt [NCLS];
  logic [3:0]       lvl [4];
  logic [CA-1:0]    ci;
  logic [1:0]       lab_l;
  logic [15:0]      dacc [NCLS];
  logic [3:0]       arc_cnt;
  logic             gated;

  logic [3:0] conf_n;
  assign conf_n = (cfg.arc_confirm == 4'd0) ? 4'd1 : cfg.arc_confirm;

  logic [63:0] hv;
  cg_hdc_enc #(.D(D), .SEED(64'hA4C0_FFEE_1234_0002), .PA(0), .PB(2)) u_enc (
    .lvl(lvl), .chunk(16'(ci)), .hv(hv)
  );

  logic [63:0]      proto [NCLS];
  logic [64*CW-1:0] upd;
  always_comb begin
    for (int c = 0; c < NCLS; c++)
      for (int k = 0; k < 64; k++) begin
        logic signed [CW-1:0] cv;
        cv = mem[c][ci][k*CW +: CW];
        proto[c][k] = (cv > 0);
      end
    for (int k = 0; k < 64; k++) begin
      logic signed [CW-1:0] cv;
      cv = mem[lab_l][ci][k*CW +: CW];
      if (hv[k]) cv = (cv == CMAX) ? cv : cv + 1'b1;
      else       cv = (cv == CMIN) ? cv : cv - 1'b1;
      upd[k*CW +: CW] = cv;
    end
  end

  // argmin over trained classes
  logic [1:0]  best;
  logic        any_tr;
  always_comb begin
    logic [15:0] bd;
    best   = 2'(CLS_BACKGND);
    bd     = 16'hFFFF;
    any_tr = 1'b0;
    for (int c = 0; c < NCLS; c++) begin
      if (ncnt[c] != 16'd0 && (!any_tr || dacc[c] < bd)) begin
        best   = 2'(c);
        bd     = dacc[c];
        any_tr = 1'b1;
      end
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      st          <= S_INIT;
      ci          <= '0;
      ready       <= 1'b0;
      done        <= 1'b0;
      cls         <= 2'(CLS_BACKGND);
      arc         <= 1'b0;
      trained_win <= 1'b0;
      overrun     <= 1'b0;
      lab_l       <= '0;
      arc_cnt     <= '0;
      gated       <= 1'b0;
      for (int c = 0; c < NCLS; c++) begin
        ncnt[c] <= '0;
        dacc[c] <= '0;
        cdist[c] <= '0;
      end
      for (int f = 0; f < 4; f++) lvl[f] <= '0;
    end else begin
      done        <= 1'b0;
      trained_win <= 1'b0;
      if (cls_reset) begin
        st      <= S_INIT;
        ci      <= '0;
        ready   <= 1'b0;
        arc     <= 1'b0;
        arc_cnt <= '0;
        for (int c = 0; c < NCLS; c++) ncnt[c] <= '0;
      end else begin
        if (f_valid && st != S_IDLE) overrun <= 1'b1;
        unique case (st)
          S_INIT: begin
            for (int c = 0; c < NCLS; c++) mem[c][ci] <= '0;
            if (ci == CA'(C - 1)) begin
              ci    <= '0;
              ready <= 1'b1;
              st    <= S_IDLE;
            end else begin
              ci <= ci + 1'b1;
            end
          end
          S_IDLE: begin
            if (f_valid) begin
              lvl[0] <= quant(32'(e),    cfg.cls_qsh[3:0]);
              lvl[1] <= quant(32'(peak), cfg.cls_qsh[7:4]);
              lvl[2] <= quant(32'(zc),   cfg.cls_qsh[11:8]);
              lvl[3] <= quant(32'(de),   cfg.cls_qsh[15:12]);
              ci     <= '0;
              for (int c = 0; c < NCLS; c++) dacc[c] <= '0;
              if (train_active && train_en && train_label < 2'(NCLS)) begin
                lab_l <= train_label;
                st    <= S_TRAIN;
              end else if (e < 24'(cfg.e_gate)) begin
                gated <= 1'b1;
                st    <= S_DONE;
              end else begin
                gated <= 1'b0;
                st    <= S_SCORE;
              end
            end
          end
          S_SCORE: begin
            for (int c = 0; c < NCLS; c++) dacc[c] <= dacc[c] + 16'(popcount64(hv ^ proto[c]));
            if (ci == CA'(C - 1)) begin
              ci <= '0;
              st <= S_DONE;
            end else begin
              ci <= ci + 1'b1;
            end
          end
          S_TRAIN: begin
            mem[lab_l][ci] <= upd;
            if (ci == CA'(C - 1)) begin
              ci           <= '0;
              ncnt[lab_l]  <= (ncnt[lab_l] == 16'hFFFF) ? ncnt[lab_l] : ncnt[lab_l] + 16'd1;
              trained_win  <= 1'b1;
              arc_cnt      <= '0;
              arc          <= 1'b0;
              cls          <= lab_l;
              done         <= 1'b1;
              st           <= S_IDLE;
            end else begin
              ci <= ci + 1'b1;
            end
          end
          S_DONE: begin
            logic [1:0] k;
            k = (gated || !any_tr) ? 2'(CLS_BACKGND) : best;
            cls  <= k;
            done <= 1'b1;
            for (int c = 0; c < NCLS; c++) cdist[c] <= dacc[c];
            if (k == 2'(CLS_ARC)) begin
              arc_cnt <= (arc_cnt == 4'hF) ? arc_cnt : arc_cnt + 4'd1;
              arc     <= ({1'b0, arc_cnt} + 5'd1 >= {1'b0, conf_n});
            end else begin
              arc_cnt <= '0;
              arc     <= 1'b0;
            end
            st <= S_IDLE;
          end
          default: st <= S_IDLE;
        endcase
      end
    end
  end

  always_comb for (int c = 0; c < NCLS; c++) cls_trained[c] = (ncnt[c] != 16'd0);

endmodule
