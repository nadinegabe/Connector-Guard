// M8: independent hard-trip protection.
// Reads the raw M0 sample stream only (no M1-M7 signal enters this module)
// and compares every sample against fixed limits:
//   |i| > i_max, |i - i_prev| > di_max, t > t_max, ires > ires_max
// plus a no-sample watchdog (armed after the first sample). Each effective
// limit is min(cfg value, *_ABS ceiling from cg_pkg), so configuration can
// tighten but never disable a limit. A violation latches trip on the next
// clock edge (latency 1 cycle from s_valid). hard_trip is also high before
// the first sample arrives, so the contactor cannot close without data.
// trip_clear releases the latch only if the last sample was within limits
// and the watchdog has not expired.
module cg_m8_hardtrip
  import cg_pkg::*;
(
  input  logic       clk,
  input  logic       rst_n,
  input  cfg_t       cfg,
  input  logic       s_valid,
  input  sample_t    s_data,
  input  logic       trip_clear,
  output logic       hard_trip,   // to the contactor gate
  output logic       trip_latched,// to M7 status (excludes the not-armed state)
  output logic [4:0] cause,       // sticky: {wdog, ires, t, di, i}
  output logic       armed
);
  logic [15:0] lim_i, lim_di, lim_t, lim_ires;
  assign lim_i    = (cfg.i_max    < I_MAX_ABS)    ? cfg.i_max    : I_MAX_ABS;
  assign lim_di   = (cfg.di_max   < DI_MAX_ABS)   ? cfg.di_max   : DI_MAX_ABS;
  assign lim_t    = (cfg.t_max    < T_MAX_ABS)    ? cfg.t_max    : T_MAX_ABS;
  assign lim_ires = (cfg.ires_max < IRES_MAX_ABS) ? cfg.ires_max : IRES_MAX_ABS;

  logic signed [15:0] i_prev;
  logic               have_prev;
  logic               trip_l;
  logic               last_ok;
  logic [15:0]        idle;
  logic               wdog;

  logic [16:0] ia, dia;
  logic signed [16:0] di;
  assign ia  = s_data.i[15] ? 17'(-17'(s_data.i)) : 17'(s_data.i);
  assign di  = 17'(s_data.i) - 17'(i_prev);
  assign dia = di[16] ? 17'(-di) : 17'(di);

  logic v_i, v_di, v_t, v_ires, viol;
  assign v_i    = ia > 17'(lim_i);
  assign v_di   = have_prev && (dia > 17'(lim_di));
  assign v_t    = 17'(s_data.t) > $signed({1'b0, lim_t});
  assign v_ires = s_data.ires > lim_ires;
  assign viol   = v_i || v_di || v_t || v_ires;

  assign wdog = armed && (idle >= cfg.stale_cycles);

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      i_prev    <= '0;
      have_prev <= 1'b0;
      trip_l    <= 1'b0;
      last_ok   <= 1'b0;
      idle      <= '0;
      armed     <= 1'b0;
      cause     <= '0;
    end else begin
      if (s_valid) begin
        i_prev    <= s_data.i;
        have_prev <= 1'b1;
        armed     <= 1'b1;
        last_ok   <= !viol;
        idle      <= '0;
      end else if (idle != 16'hFFFF) begin
        idle <= idle + 16'd1;
      end

      if (s_valid && viol) begin
        trip_l <= 1'b1;
        cause  <= cause | {1'b0, v_ires, v_t, v_di, v_i};
      end else if (wdog) begin
        trip_l <= 1'b1;
        cause  <= cause | 5'b10000;
      end else if (trip_clear && armed && (s_valid || last_ok)) begin
        trip_l <= 1'b0;
        cause  <= '0;
      end
    end
  end

  assign hard_trip    = trip_l || !armed;
  assign trip_latched = trip_l;

endmodule
