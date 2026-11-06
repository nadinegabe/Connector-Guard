// Sequential restoring divider (unsigned), one quotient bit per cycle.
// quot = min(num / den, 2^QW - 1). den == 0 returns all-ones and sets div0.
// Latency: NW + 1 cycles from start to done.
module cg_div #(
  parameter int NW = 40,
  parameter int DW = 24,
  parameter int QW = 16
) (
  input  logic          clk,
  input  logic          rst_n,
  input  logic          start,
  input  logic [NW-1:0] num,
  input  logic [DW-1:0] den,
  output logic          busy,
  output logic          done,
  output logic [QW-1:0] quot,
  output logic          div0
);
  localparam int CW = $clog2(NW + 1);

  logic [NW-1:0] q_r;
  logic [NW-1:0] n_r;
  logic [DW:0]   rem_r;
  logic [DW-1:0] d_r;
  logic [CW-1:0] cnt;

  logic [DW:0] rem_sh;
  logic [DW:0] rem_sub;
  assign rem_sh  = {rem_r[DW-1:0], n_r[NW-1]};
  assign rem_sub = rem_sh - {1'b0, d_r};

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      busy  <= 1'b0;
      done  <= 1'b0;
      quot  <= '0;
      div0  <= 1'b0;
      q_r   <= '0;
      n_r   <= '0;
      rem_r <= '0;
      d_r   <= '0;
      cnt   <= '0;
    end else begin
      done <= 1'b0;
      if (start && !busy) begin
        if (den == '0) begin
          quot <= '1;
          div0 <= 1'b1;
          done <= 1'b1;
        end else begin
          busy  <= 1'b1;
          div0  <= 1'b0;
          n_r   <= num;
          d_r   <= den;
          rem_r <= '0;
          q_r   <= '0;
          cnt   <= CW'(NW);
        end
      end else if (busy) begin
        n_r <= {n_r[NW-2:0], 1'b0};
        if (!rem_sub[DW]) begin
          rem_r <= rem_sub;
          q_r   <= {q_r[NW-2:0], 1'b1};
        end else begin
          rem_r <= rem_sh;
          q_r   <= {q_r[NW-2:0], 1'b0};
        end
        cnt <= cnt - 1'b1;
        if (cnt == CW'(1)) begin
          busy <= 1'b0;
          done <= 1'b1;
          if (!rem_sub[DW]) begin
            quot <= sat({q_r[NW-2:0], 1'b1});
          end else begin
            quot <= sat({q_r[NW-2:0], 1'b0});
          end
        end
      end
    end
  end

  function automatic logic [QW-1:0] sat(input logic [NW-1:0] q);
    if (q > NW'({QW{1'b1}})) return '1;
    return q[QW-1:0];
  endfunction

endmodule
