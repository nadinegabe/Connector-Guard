// M7: severity fusion and 4-state FSM (NORMAL < WATCH < DERATE < TRIP).
//
// Requested severity = max(latest M5a request, latest M5b request,
//                          TRIP if M6 arc, DERATE if sensor fault or stale,
//                          TRIP if M8 hard trip).
// Rules:
//   * escalation to the requested severity is immediate (same cycle)
//   * de-escalation from NORMAL..DERATE happens one level at a time, only
//     after hold_windows consecutive window ticks whose request is below the
//     current state; any tick at or above the state restarts the count
//   * TRIP is latched; it is left only on trip_clear while the requested
//     severity is below TRIP, and then goes to DERATE (not NORMAL)
// There is no input that lowers the state directly: AI requests can only
// raise it, and the absence of requests only lowers it through the
// hysteresis rule above.
module cg_m7_fsm
  import cg_pkg::*;
(
  input  logic       clk,
  input  logic       rst_n,
  input  cfg_t       cfg,
  input  logic       a_valid,       // M5a done (also the window tick)
  input  logic [1:0] a_req,
  input  logic       b_valid,       // M5b done
  input  logic [1:0] b_req,
  input  logic       c_valid,       // M6 done
  input  logic       c_arc,
  input  logic       sensor_fault,
  input  logic       stale,
  input  logic       hard_trip,
  input  logic       trip_clear,
  output logic [1:0] state,
  output logic [1:0] req_now,
  output logic       fsm_trip,
  output logic       learn_ok,
  output logic       clear_rejected
);
  logic [1:0] la, lb;
  logic       larc;
  logic [7:0] below;

  logic [1:0] a_e, b_e;
  logic       arc_e;
  assign a_e   = a_valid ? a_req : la;
  assign b_e   = b_valid ? b_req : lb;
  assign arc_e = c_valid ? c_arc : larc;

  always_comb begin
    logic [1:0] r;
    r = sev_max(a_e, b_e);
    if (sensor_fault || stale) r = sev_max(r, SEV_DERATE);
    if (arc_e || hard_trip)    r = SEV_TRIP;
    req_now = r;
  end

  logic [7:0] hold_n;
  assign hold_n = (cfg.hold_windows == 8'd0) ? 8'd1 : cfg.hold_windows;

  always_ff @(posedge clk or negedge rst_n) begin
    if (!rst_n) begin
      state          <= SEV_NORMAL;
      la             <= SEV_NORMAL;
      lb             <= SEV_NORMAL;
      larc           <= 1'b0;
      below          <= '0;
      clear_rejected <= 1'b0;
    end else begin
      la             <= a_e;
      lb             <= b_e;
      larc           <= arc_e;
      clear_rejected <= 1'b0;
      if (req_now > state) begin
        state <= req_now;
        below <= '0;
      end else if (state == SEV_TRIP) begin
        below <= '0;
        if (trip_clear) begin
          if (req_now < SEV_TRIP) state <= SEV_DERATE;
          else                    clear_rejected <= 1'b1;
        end
      end else if (a_valid) begin
        if (req_now < state) begin
          if (below + 8'd1 >= hold_n) begin
            state <= state - 2'd1;
            below <= '0;
          end else begin
            below <= below + 8'd1;
          end
        end else begin
          below <= '0;
        end
      end
    end
  end

  assign fsm_trip = (state == SEV_TRIP);
  assign learn_ok = (state == SEV_NORMAL);

endmodule
