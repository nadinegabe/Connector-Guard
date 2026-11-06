// HDC encoder (combinational, one 64-bit chunk of the hypervector).
//   b_f  = ID_f XOR LEVEL_f(lvl_f)                 for f = 0..3
//   pair = b_PA XOR b_PB XOR ID_pair               interaction term
//   H    = majority(b_0, b_1, b_2, b_3, pair)      five terms, no ties
// ID and level-base words come from cg_pkg::im_word(seed, chunk), so no
// item memory is stored. LEVEL_f(L) flips the first L*STEP bits of the
// level base, STEP = D / (2*(QL-1)): neighbouring levels stay similar and
// level 0 and level 15 differ in about D/2 bits.
module cg_hdc_enc
  import cg_pkg::*;
#(
  parameter int          D    = 1024,
  parameter logic [63:0] SEED = 64'h5EED_C0DE_0000_0001,
  parameter int          PA   = 0,
  parameter int          PB   = 2
) (
  input  logic [3:0]  lvl [4],
  input  logic [15:0] chunk,
  output logic [63:0] hv
);
  localparam int STEP = D / (2 * (QL - 1));

  function automatic logic [63:0] id_seed(input int f);
    return SEED ^ (64'h0123_4567_89AB_CDEF * 64'(f + 1));
  endfunction
  function automatic logic [63:0] lv_seed(input int f);
    return SEED ^ (64'hF0E1_D2C3_B4A5_9687 * 64'(f + 1)) ^ 64'hA5A5_5A5A_A5A5_5A5A;
  endfunction

  logic [63:0] b [4];
  logic [63:0] pair;

  always_comb begin
    for (int f = 0; f < 4; f++) begin
      logic [63:0] id, lb, mask;
      logic [31:0] thr;
      id  = im_word(id_seed(f), chunk);
      lb  = im_word(lv_seed(f), chunk);
      thr = 32'(lvl[f]) * 32'(STEP);
      for (int k = 0; k < 64; k++) mask[k] = ((32'(chunk) * 32'd64 + 32'(k)) < thr);
      b[f] = id ^ lb ^ mask;
    end
    pair = b[PA] ^ b[PB] ^ im_word(id_seed(7), chunk);
    for (int k = 0; k < 64; k++) begin
      logic [2:0] n;
      n = 3'(b[0][k]) + 3'(b[1][k]) + 3'(b[2][k]) + 3'(b[3][k]) + 3'(pair[k]);
      hv[k] = (n >= 3'd3);
    end
  end

endmodule
