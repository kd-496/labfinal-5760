`timescale 1ns / 1ps

module DE1_SoC_Computer (
    // [Existing ports omitted for brevity]

    // Clock pins
    input CLOCK_50,

    // Pushbuttons
    input [3:0] KEY,

    // VGA
    output [7:0] VGA_R, VGA_G, VGA_B,
    output VGA_HS, VGA_VS,

    // [Other ports omitted for brevity]
);

    // Parameters for orbital mechanics (fixed-point representation)
    localparam G = 32'd671430;  // Adjusted gravitational constant
    localparam M = 32'd597200000;  // Adjusted mass of Earth
    localparam dt = 32'd10;  // Time step in clock cycles

    // Registers for position and velocity (fixed-point)
    reg [31:0] x, y, vx, vy;

    // Registers for acceleration (fixed-point)
    reg [31:0] ax, ay;

    // VGA display wires
    wire [9:0] pixel_x, pixel_y;
    wire pixel_clk;

    // Initial conditions
    initial begin
        x = 32'd6371000 + 32'd400000;  // Position x (radius of Earth + altitude)
        y = 32'd0;                     // Position y
        vx = 32'd0;                    // Velocity x
        vy = 32'd7758;                 // Velocity y (orbital velocity at 400 km altitude)
    end

    // Orbital mechanics calculation at each time step
    always @(posedge CLOCK_50) begin
        if (!KEY[0]) begin
            // Reset to initial conditions when KEY[0] is pressed
            x <= 32'd6371000 + 32'd400000;
            y <= 32'd0;
            vx <= 32'd0;
            vy <= 32'd7758;
        end else begin
            // Calculate radial distance
            reg [63:0] r_squared = x * x + y * y;
            reg [31:0] r_cubed = r_squared * $signed($sqrt(r_squared));

            // Compute acceleration
            ax = -G * M * x / r_cubed;
            ay = -G * M * y / r_cubed;

            // Update velocities
            vx = vx + ax * dt;
            vy = vy + ay * dt;

            // Update positions
            x = x + vx * dt;
            y = y + vy * dt;
        end
    end

    // Map calculated positions to VGA coordinates
    assign pixel_x = (x >> 12) % 640;  // Scale and wrap around screen width
    assign pixel_y = (y >> 12) % 480;  // Scale and wrap around screen height

    // Instantiate VGA driver
    vga_driver vga_display (
        .clock(CLOCK_50),
        .reset_n(KEY[0]),
        .pixel_x(pixel_x),
        .pixel_y(pixel_y),
        .VGA_R(VGA_R),
        .VGA_G(VGA_G),
        .VGA_B(VGA_B),
        .VGA_HS(VGA_HS),
        .VGA_VS(VGA_VS)
    );

endmodule

module vga_driver(
    input clock,
    input reset_n,
    input [9:0] pixel_x, pixel_y,
    output reg [7:0] VGA_R, VGA_G, VGA_B,
    output reg VGA_HS, VGA_VS
);
    // [VGA signal generation logic here, as previously defined]
endmodule
