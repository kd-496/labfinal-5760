module Orbital_Path (
    input clk,           // Clock input
    input rst,           // Reset input
    output reg [26:0] X, // X position output
    output reg [26:0] Y  // Y position output
);

// Constants and initial conditions
parameter G = 27'h0000005D;  // Adjusted gravitational constant for simulation
parameter M = 27'h00000005;  // Adjusted mass of Earth for simulation
parameter DT = 27'h0000000A;  // Time step in seconds
parameter steps = 10;        // Total simulation steps

// State variables for position and velocity (floating-point registers)
reg [26:0] x_fp, y_fp, vx_fp, vy_fp;
reg [26:0] ax_fp, ay_fp;

// Calculation of radius squared and inverse cube of radius (floating-point wires)
wire [26:0] r_squared_fp, inv_r_cubed_fp;
FpMul r_squared_x(.iA(x_fp), .iB(x_fp), .oProd(x_fp_squared));  // x squared
FpMul r_squared_y(.iA(y_fp), .iB(y_fp), .oProd(y_fp_squared));  // y squared
FpAdd r_squared_calc(.iA(x_fp_squared), .iB(y_fp_squared), .oSum(r_squared_fp));  // x^2 + y^2
FpInvSqrt inv_sqrt_r_cubed(.iA(r_squared_fp), .oInvSqrt(inv_r_cubed_fp));  // 1 / sqrt(r^3)

// Calculation of accelerations based on gravitational force
always @(posedge clk or posedge rst) begin
    if (rst) begin
        // Initialize positions and velocities
        x_fp <= 27'd100000;  // Initial x position
        y_fp <= 27'd0;       // Initial y position
        vx_fp <= 27'd0;      // Initial x velocity
        vy_fp <= 27'd100;    // Initial y velocity
    end else begin
        // Compute accelerations using floating-point operations
        FpMul ax_calc(.iA(x_fp), .iB(inv_r_cubed_fp), .oProd(ax_fp_temp));
        FpMul ay_calc(.iA(y_fp), .iB(inv_r_cubed_fp), .oProd(ay_fp_temp));
        FpMul ax_final(.iA(ax_fp_temp), .iB(G * M), .oProd(ax_fp));
        FpMul ay_final(.iA(ay_fp_temp), .iB(G * M), .oProd(ay_fp));

        // Update velocities
        FpAdd vx_update(.iA(vx_fp), .iB(ax_fp * DT), .oSum(vx_fp));
        FpAdd vy_update(.iA(vy_fp), .iB(ay_fp * DT), .oSum(vy_fp));

        // Update positions
        FpAdd x_update(.iA(x_fp), .iB(vx_fp * DT), .oSum(x_fp));
        FpAdd y_update(.iA(y_fp), .iB(vy_fp * DT), .oSum(y_fp));
    end
end

// Output the final positions
always @(posedge clk) begin
    X <= x_fp;
    Y <= y_fp;
end

endmodule
