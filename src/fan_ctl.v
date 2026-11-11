// fan_ctl -- the fan's PWM from the die's temperature and a floor that software sets.
//
// The fan runs at the larger of two duties, in percent: the temperature's -- 0 up to 40 C, then 3
// per degree, 100 from 74 C -- and the floor (the board page's FAN register, 0x1000_6008). So
// software can make the fan louder, never quieter than the die needs. Everything that decides the
// duty runs on clk, the board's own oscillator, so the fan follows the die from configuration on,
// whatever the memory controller or the core are doing. With no temperature yet, or none for
// STALE periods, the duty is 100.
//
// The temperature is SYSMONE4's code (C = code x 509.314 / 65536 - 280.231); t_v marks a new one.
// A new duty starts with a PWM period, so a period is never cut.
//
// The register side runs on rclk. Each direction crosses with a held value and a toggle: the sender
// changes its value only once the receiver has acknowledged the last toggle, so the value is still
// when the toggle reaches the receiver two flops later.
module fan_ctl #(
   parameter integer PERIOD = 2000,         // clk cycles a PWM period: 50 MHz / 2000 = 25 kHz
   parameter integer STALE  = 2500          // PWM periods with no temperature before full duty (0.1 s)
) (
   input  wire        clk,
   input  wire        t_v,
   input  wire [15:0] t_code,
   output reg         fan_on = 1'b1,        // the fan is powered this cycle
   input  wire        rclk,
   input  wire [ 6:0] r_floor,              // percent (above 100 is 100), in rclk
   output reg  [22:0] r_stat = 23'd0        // {duty now, the temperature code}, in rclk
);
   localparam [15:0] C40   = 16'd41206;     // the code at 40 C
   localparam [10:0] SLOPE = 11'd1528;      // 3 percent a degree: 3 x 65536 / 128.675 codes a degree
   localparam integer PW = $clog2(PERIOD);
   localparam integer SW = $clog2(STALE + 1);

   // ---- the floor, rclk -> clk ----
   reg  [6:0] f_h = 7'd100;  reg f_t = 1'b0;                      // rclk: the held value and its toggle
   (* async_reg = "true" *) reg f_a0 = 1'b0, f_a1 = 1'b0;         // rclk: clk's acknowledgement
   (* async_reg = "true" *) reg f_s0 = 1'b0, f_s1 = 1'b0;         // clk: the toggle
   reg        f_k = 1'b0;                                         // clk: the toggle taken
   reg  [6:0] floor_c = 7'd100;
   always @(posedge rclk) begin
      f_a0 <= f_k;  f_a1 <= f_a0;
      if (f_a1 == f_t && r_floor != f_h) begin f_h <= r_floor; f_t <= ~f_t; end
   end
   always @(posedge clk) begin
      f_s0 <= f_t;  f_s1 <= f_s0;
      if (f_s1 != f_k) begin floor_c <= f_h; f_k <= f_s1; end
   end

   // ---- the duty ----
   reg  [15:0] t_q = 16'd0;
   reg  [SW-1:0] age = STALE[SW-1:0];       // PWM periods since the last temperature, held at STALE
   wire [15:0] over  = (t_q > C40) ? t_q - C40 : 16'd0;
   wire [26:0] prod  = over * SLOPE;
   wire [ 6:0] d_t   = (prod[26:16] >= 11'd100) ? 7'd100 : prod[22:16];
   wire [ 6:0] d_f   = (floor_c > 7'd100) ? 7'd100 : floor_c;
   wire [ 6:0] d_new = (age == STALE[SW-1:0]) ? 7'd100 : (d_f > d_t ? d_f : d_t);
   reg  [ 6:0] duty = 7'd100;
   reg  [PW-1:0] cnt = {PW{1'b0}};
   wire        p_end = cnt == PERIOD[PW-1:0] - 1'b1;
   always @(posedge clk) begin
      if (t_v) t_q <= t_code;
      cnt <= p_end ? {PW{1'b0}} : cnt + 1'b1;
      if (t_v) age <= {SW{1'b0}};
      else if (p_end && age != STALE[SW-1:0]) age <= age + 1'b1;
      if (p_end) duty <= d_new;
      fan_on <= cnt < duty * (PERIOD / 100);
   end

   // ---- the status, clk -> rclk ----
   reg  [22:0] s_h = 23'd0;  reg s_t = 1'b0;                      // clk: the held value and its toggle
   (* async_reg = "true" *) reg s_a0 = 1'b0, s_a1 = 1'b0;         // clk: rclk's acknowledgement
   (* async_reg = "true" *) reg s_s0 = 1'b0, s_s1 = 1'b0;         // rclk: the toggle
   reg         s_k = 1'b0;                                        // rclk: the toggle taken
   always @(posedge clk) begin
      s_a0 <= s_k;  s_a1 <= s_a0;
      if (s_a1 == s_t && {duty, t_q} != s_h) begin s_h <= {duty, t_q}; s_t <= ~s_t; end
   end
   always @(posedge rclk) begin
      s_s0 <= s_t;  s_s1 <= s_s0;
      if (s_s1 != s_k) begin r_stat <= s_h; s_k <= s_s1; end
   end

   // ---- invariants ----
   always @(posedge clk) begin
      if (duty > 7'd100) $fatal(1, "fan_ctl: a duty of %0d percent", duty);
      if (PERIOD % 100 != 0) $fatal(1, "fan_ctl: PERIOD %0d is not a multiple of 100", PERIOD);
   end
endmodule
