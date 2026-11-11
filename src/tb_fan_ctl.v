`timescale 1ns/1ps
`default_nettype none

// Unit test for fan_ctl.v at a 200-cycle PWM period: full duty before any temperature and once
// the temperature goes stale, the temperature's duty (0 to 40 C, 3 a degree, 100 from 74 C), the
// floor raising it and never lowering it, a floor above 100, and the status crossing back.
// clk and rclk are unrelated (10 ns and 3.3 ns). Prints "FAN_CTL: PASS" or the failures.
module tb;
   localparam integer PERIOD = 200, STALE = 4;
   reg clk = 0;   always #5    clk  = ~clk;
   reg rclk = 0;  always #1.65 rclk = ~rclk;

   reg         t_v = 0;
   reg  [15:0] t_code = 0;
   reg  [ 6:0] r_floor = 7'd30;
   wire        fan_on;
   wire [22:0] r_stat;

   fan_ctl #(.PERIOD(PERIOD), .STALE(STALE)) dut (.clk(clk), .t_v(t_v), .t_code(t_code), .fan_on(fan_on),
      .rclk(rclk), .r_floor(r_floor), .r_stat(r_stat));

   integer fails = 0;

   // the high cycles of fan_on over one PWM period, a few periods after the last change
   task duty_is(input integer pct, input [8*40-1:0] what);
      integer i, on;
      begin
         repeat (3 * PERIOD) @(posedge clk);
         on = 0;
         for (i = 0; i < PERIOD; i = i + 1) begin @(posedge clk); on = on + fan_on; end
         if (on != pct * PERIOD / 100) begin
            $display("  FAIL: %0s: %0d of %0d cycles on, want %0d percent", what, on, PERIOD, pct);
            fails = fails + 1;
         end
         repeat (20) @(posedge rclk);
         if (r_stat[22:16] != pct[6:0]) begin
            $display("  FAIL: %0s: the status says %0d percent", what, r_stat[22:16]);
            fails = fails + 1;
         end
      end
   endtask

   // a temperature reading every 64 clk cycles while `feed` is on
   reg feed = 0;
   always @(posedge clk) begin
      t_v <= feed && ($time / 10) % 64 == 0;
   end

   initial begin
      duty_is(100, "no temperature yet");
      t_code = 16'd40356;  feed = 1;                  // 33.4 C: 0, so the floor's 30
      duty_is(30, "33.4 C, floor 30");
      if (r_stat[15:0] != 16'd40356) begin $display("  FAIL: status code %0d", r_stat[15:0]); fails = fails + 1; end
      t_code = 16'd43136;  duty_is(44, "55 C, floor 30");
      t_code = 16'd43779;  duty_is(59, "60 C, floor 30");
      r_floor = 7'd75;     duty_is(75, "60 C, floor 75");
      t_code = 16'd45581;  duty_is(100, "74 C, floor 75");
      t_code = 16'd46353;  duty_is(100, "80 C, floor 75");
      t_code = 16'd40356;  r_floor = 7'd0;  duty_is(0, "33.4 C, floor 0");
      r_floor = 7'd120;    duty_is(100, "a floor of 120");
      r_floor = 7'd20;     duty_is(20, "33.4 C, floor 20");
      feed = 0;            repeat ((STALE + 2) * PERIOD) @(posedge clk);
      duty_is(100, "the temperature stale");
      feed = 1;            duty_is(20, "the temperature back");
      if (fails == 0) $display("FAN_CTL: PASS");
      else            $display("FAN_CTL: FAIL (%0d)", fails);
      $finish;
   end

   initial begin #50_000_000; $display("FAN_CTL: FAIL (timeout)"); $finish; end
endmodule
