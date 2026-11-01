// ConnectorGuard-HDC shared package: sample format, severity encoding,
// configuration record with reset defaults, and HDC item-memory helpers.
//
// Fixed-point conventions (all ADC codes are already linearised by the AFE):
//   i     : signed 16, 10 mA / LSB      (load current, charging direction > 0)
//   v     : signed 16, 10 uV / LSB      (4-wire contact voltage drop)
//   t     : signed 16, 1/16 degC / LSB  (connector NTC temperature)
//   tamb  : signed 16, 1/16 degC / LSB  (ambient temperature)
//   ires  : unsigned 16, 0.1 mA / LSB   (DC residual current)
//   R     : unsigned 16, 1/1024 mOhm / LSB (~0.98 uOhm)
package cg_pkg;

  typedef struct packed {
    logic signed [15:0] i;
    logic signed [15:0] v;
    logic signed [15:0] t;
    logic signed [15:0] tamb;
    logic        [15:0] ires;
  } sample_t;

  typedef enum logic [1:0] {
    SEV_NORMAL = 2'd0,
    SEV_WATCH  = 2'd1,
    SEV_DERATE = 2'd2,
    SEV_TRIP   = 2'd3
  } sev_t;

  // M6 classes
  localparam int NCLS        = 3;
  localparam int CLS_BACKGND = 0;
  localparam int CLS_SWITCH  = 1;
  localparam int CLS_ARC     = 2;

  // HDC constants
  localparam int HW = 64;   // bits processed per cycle (one chunk)
  localparam int QL = 16;   // quantisation levels per feature

  // Context bins: 4 current bins x 2 ambient bins
  localparam int NB = 8;

  // Absolute hard-trip ceilings. Configuration can tighten M8 thresholds but
  // never loosen them past these values.
  localparam logic [15:0] I_MAX_ABS    = 16'd30000;  // 300 A
  localparam logic [15:0] DI_MAX_ABS   = 16'd8000;   // 80 A per sample
  localparam logic [15:0] T_MAX_ABS    = 16'd1760;   // 110 degC
  localparam logic [15:0] IRES_MAX_ABS = 16'd300;    // 30 mA
  localparam logic [31:0] STALE_ABS    = 32'd65535;  // watchdog ceiling

  typedef struct packed {
    // window / learning
    logic [3:0]  win_log2;      // window = 2^win_log2 samples, clamped 3..10
    logic        learn_en;
    logic [15:0] stale_cycles;  // no-sample watchdog (M0 fault, M8 trip)
    // M8 hard-trip thresholds
    logic [15:0] i_max;
    logic [15:0] di_max;
    logic [15:0] t_max;
    logic [15:0] ires_max;
    // M1 NTC plausibility window
    logic signed [15:0] t_lo;
    logic signed [15:0] t_hi;
    // M3 / M4
    logic [15:0] i_min_r;       // minimum window current for valid R / thermal index
    logic [7:0]  alpha_t;       // R temperature coefficient, Q16 per (1/16 degC)
    logic signed [15:0] tref;   // R compensation reference temperature
    // M5a
    logic [3:0]  ewma_shift;
    logic [3:0]  drift_shift;   // drift watch if mean_a - mean_f > mean_f >> drift_shift
    logic [7:0]  k_watch;       // deviation thresholds in quarter-MAD units
    logic [7:0]  k_derate;
    logic [7:0]  mad_floor;
    logic [7:0]  hold_windows;  // M7 de-escalation hysteresis
    // M5b / M6
    logic [7:0]  freeze_n;      // learning updates before a bin is trained/frozen
    logic [7:0]  forget_period; // counter halving period (updates), 0 = off
    logic [7:0]  k_novel;       // novelty threshold in quarter-MAD units
    logic [3:0]  arc_confirm;   // consecutive ARC windows before trip
    logic [15:0] hdc_qsh;       // M5b feature quantiser shifts, 4 x 4 bits
    logic [15:0] cls_qsh;       // M6 feature quantiser shifts, 4 x 4 bits
    logic [15:0] e_gate;        // M6 minimum HF energy to classify
    logic [15:0] hdc_drift_thr; // M5b drift: d_frozen - d_adapt > thr
    // context bins
    logic [3:0]  ibin_shift;
    logic signed [15:0] tbin_thr;
    // M9 CP PWM duty codes (0..255)
    logic [7:0]  duty_normal;
    logic [7:0]  duty_derate;
  } cfg_t;

  function automatic cfg_t cfg_default();
    cfg_t c;
    c.win_log2      = 4'd6;
    c.learn_en      = 1'b1;
    c.stale_cycles  = 16'd4000;
    c.i_max         = 16'd25000;
    c.di_max        = 16'd4000;
    c.t_max         = 16'd1440;     // 90 degC
    c.ires_max      = 16'd60;       // 6 mA
    c.t_lo          = -16'sd640;    // -40 degC
    c.t_hi          = 16'sd2400;    // 150 degC
    c.i_min_r       = 16'd1000;     // 10 A
    c.alpha_t       = 8'd16;        // ~0.39 %/degC (copper)
    c.tref          = 16'sd400;     // 25 degC
    c.ewma_shift    = 4'd4;
    c.drift_shift   = 4'd3;         // 12.5 %
    c.k_watch       = 8'd16;        // 4 MAD
    c.k_derate      = 8'd32;        // 8 MAD
    c.mad_floor     = 8'd4;
    c.hold_windows  = 8'd8;
    c.freeze_n      = 8'd16;
    c.forget_period = 8'd64;
    c.k_novel       = 8'd24;        // 6 MAD
    c.arc_confirm   = 4'd2;
    c.hdc_qsh       = 16'h0477;     // {ires,E,therm,R} shifts
    c.cls_qsh       = 16'h3388;     // {dE,zc,peak,E} shifts
    c.e_gate        = 16'd64;
    c.hdc_drift_thr = 16'd48;
    c.ibin_shift    = 4'd12;        // 4096 codes ~ 41 A per bin
    c.tbin_thr      = 16'sd480;     // 30 degC
    c.duty_normal   = 8'd136;       // ~53 %
    c.duty_derate   = 8'd64;        // ~25 %
    return c;
  endfunction

  function automatic logic [1:0] sev_max(input logic [1:0] a, input logic [1:0] b);
    return (a > b) ? a : b;
  endfunction

  // ---------------------------------------------------------------------
  // HDC item memory: a pure function of (seed, chunk index). Built from a
  // constant multiply and xorshift rounds, so ID and level hypervectors are
  // regenerated on the fly instead of being stored.
  // ---------------------------------------------------------------------
  function automatic logic [63:0] im_word(input logic [63:0] seed, input logic [15:0] idx);
    logic [63:0] x;
    logic [127:0] p;
    p = {64'd0, 48'd0, idx} * 128'h9E3779B97F4A7C15;
    x = seed ^ p[63:0] ^ 64'hD1B54A32D192ED03;
    x = x ^ (x << 13);
    x = x ^ (x >> 7);
    x = x ^ (x << 17);
    x = x ^ (x << 13);
    x = x ^ (x >> 7);
    x = x ^ (x << 17);
    return x;
  endfunction

  function automatic logic [3:0] quant(input logic [31:0] x, input logic [3:0] sh);
    logic [31:0] q;
    q = x >> sh;
    return (q > 32'd15) ? 4'd15 : q[3:0];
  endfunction

  function automatic logic [6:0] popcount64(input logic [63:0] x);
    logic [6:0] n;
    n = '0;
    for (int k = 0; k < 64; k++) n = n + {6'd0, x[k]};
    return n;
  endfunction

  // Context bin {tbin, ibin[1:0]}
  function automatic logic [2:0] ctx_bin(input logic signed [15:0] i_avg,
                                         input logic signed [15:0] tamb_avg,
                                         input logic [3:0] ibin_shift,
                                         input logic signed [15:0] tbin_thr);
    logic [15:0] ia;
    logic [15:0] ib;
    ia = (i_avg < 0) ? 16'd0 : i_avg;
    ib = ia >> ibin_shift;
    return {(tamb_avg > tbin_thr), (ib > 16'd3) ? 2'd3 : ib[1:0]};
  endfunction

endpackage
