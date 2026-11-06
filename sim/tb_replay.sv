// Simulasi cg_replay_top: baca stimulus (hex, 4 word per baris) dan tulis log window (8 word per baris).
module tb_replay;
  logic clk = 0;
  always #5 clk = ~clk;
  logic rst_n = 0;
  logic [31:0]  ctrl = 0;
  logic [31:0]  stat;
  logic [127:0] s_tdata = '0; logic s_tvalid = 0, s_tready, s_tlast = 0;
  logic [255:0] m_tdata;      logic m_tvalid, m_tlast;
  logic m_tready = 1;
  logic contactor_close, hard_trip, cp_pwm, heartbeat;
  logic [1:0] state;

  cg_replay_top dut (.clk, .rst_n, .ctrl, .stat,
    .s_axis_tdata(s_tdata), .s_axis_tvalid(s_tvalid), .s_axis_tready(s_tready), .s_axis_tlast(s_tlast),
    .m_axis_tdata(m_tdata), .m_axis_tvalid(m_tvalid), .m_axis_tready(m_tready), .m_axis_tlast(m_tlast),
    .contactor_close, .state, .hard_trip, .cp_pwm, .heartbeat);

  integer fi, fo, r, nbeat;
  logic [31:0] a, b, c, d;
  string in_path, out_path;
  logic [127:0] mem [0:60000];
  integer n, k;

  // log writer
  always @(posedge clk) if (m_tvalid && m_tready) begin
    for (int w = 0; w < 8; w++) $fwrite(fo, "%08x%s", m_tdata[32*w +: 32], (w == 7) ? "\n" : " ");
  end

  initial begin
    if (!$value$plusargs("in=%s", in_path))   in_path  = "stim.hex";
    if (!$value$plusargs("out=%s", out_path)) out_path = "log.txt";
    fi = $fopen(in_path, "r"); fo = $fopen(out_path, "w");
    n = 0;
    while (!$feof(fi)) begin
      r = $fscanf(fi, "%h %h %h %h\n", a, b, c, d);
      if (r != 4) break;
      mem[n] = {d, c, b, a}; n++;
    end
    $fclose(fi);
    repeat (4) @(negedge clk);
    rst_n = 1;
    repeat (4) @(negedge clk);
    ctrl[0] = 1;                       // go
    k = 0;
    while (k < n) begin
      @(negedge clk);
      s_tvalid = 1; s_tdata = mem[k]; s_tlast = (k == n - 1);
      @(posedge clk);
      if (s_tready) k++;
    end
    @(negedge clk); s_tvalid = 0; s_tlast = 0;
    wait (stat[2] === 1'b1);
    repeat (50) @(negedge clk);
    $fclose(fo);
    $display("TB_REPLAY done beats=%0d stat=%08x", n, stat);
    $finish;
  end
endmodule
