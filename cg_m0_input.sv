// M0: sample source (live ADC stream or replay buffer), window framing and
// stale-input detection.
//
// Live samples are accepted whenever live_valid is high (live_ready = 1).
// Replay samples are written through rp_wr_* and played back once rp_start
// pulses, one sample every rp_period clocks, rp_len samples in total.
// win_last marks the last accepted sample of each 2^win_log2 window.
module cg_m0_input
  import cg_pkg::*;
#(
  parameter int RP_DEPTH = 1024
) (
  input  logic        clk,
  input  logic        rst_n,
  input  cfg_t        cfg,
  // live ADC stream
  input  logic        live_valid,
  input  sample_t     live_data,
  output logic        live_ready,
  // replay buffer
  input  logic        src_sel,      // 0 = live, 1 = replay
  input  logic        rp_wr_en,
  input  logic [$clog2(RP_DEPTH)-1:0] rp_wr_addr,
  input  sample_t     rp_wr_data,
  input  logic        rp_start,
  input  logic [$clog2(RP_DEPTH):0]   rp_len,
  input  logic [15:0] rp_period,
  output logic        rp_busy,
  // sample stream out
  output logic        s_valid,
  output sample_t     s_data,
  output logic        s_win_last,
  output logic        stale
);
  localparam int AW = $clog2(RP_DEPTH);

  sample_t      rp_mem [RP_DEPTH];
  logic [AW:0]  rp_ptr;
  logic [AW:0]  rp_len_r;
  logic [15:0]  rp_tmr;

  logic [3:0]   wl;
  logic [9:0]   win_cnt;
  logic [9:0]   win_mask;
  logic [15:0]  idle_cnt;

  logic         in_valid;
  sample_t      in_data;

  assign live_ready = 1'b1;
  assign wl         = (cfg.win_log2 < 4'd3) ? 4'd3 : ((cfg.win_log2 > 4'd10) ? 4'd10 : cfg.win_log2);
  assign win_mask   = 10'((11'd1 << wl) - 11'd1);

  always_ff @(posedge clk) begin
    if (rp_wr_en) rp_mem[rp_wr_addr] <= rp_wr_data;
  end

  // replay sequencer
  logic rp_fire;
  assign rp_fire = rp_busy && (rp_tmr == 16'd0);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      rp_busy  <= 1'b0;
      rp_ptr   <= '0;
      rp_len_r <= '0;
      rp_tmr   <= '0;
    end else if (rp_start && !rp_busy && rp_len != '0) begin
      rp_busy  <= 1'b1;
      rp_ptr   <= '0;
      rp_len_r <= rp_len;
      rp_tmr   <= 16'd0;
    end else if (rp_busy) begin
      if (rp_fire) begin
        rp_tmr <= (rp_period == 16'd0) ? 16'd0 : rp_period - 16'd1;
        rp_ptr <= rp_ptr + 1'b1;
        if (rp_ptr + 1'b1 == rp_len_r) rp_busy <= 1'b0;
      end else begin
        rp_tmr <= rp_tmr - 16'd1;
      end
    end
  end

  always_comb begin
    if (src_sel) begin
      in_valid = rp_fire && src_sel;
      in_data  = rp_mem[rp_ptr[AW-1:0]];
    end else begin
      in_valid = live_valid;
      in_data  = live_data;
    end
  end

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      s_valid    <= 1'b0;
      s_data     <= '0;
      s_win_last <= 1'b0;
      win_cnt    <= '0;
      idle_cnt   <= '0;
      stale      <= 1'b0;
    end else begin
      s_valid    <= in_valid;
      s_win_last <= in_valid && ((win_cnt & win_mask) == win_mask);
      if (in_valid) begin
        s_data  <= in_data;
        win_cnt <= ((win_cnt & win_mask) == win_mask) ? 10'd0 : win_cnt + 10'd1;
      end
      if (in_valid) begin
        idle_cnt <= '0;
        stale    <= 1'b0;
      end else if (idle_cnt >= cfg.stale_cycles) begin
        stale <= 1'b1;
      end else begin
        idle_cnt <= idle_cnt + 16'd1;
      end
    end
  end

endmodule
