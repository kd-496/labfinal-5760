module Orbital_Path (
    input clk,            // Clock input
    input rst,            // Reset input
    output reg [26:0] X,  // X position output
    output reg [26:0] Y   // Y position output
);

// Constants and initial conditions
parameter G = 27'h0000005D;  // Gravitational constant in m^3 kg^-1 s^-2 (6.67430e-11)
parameter M = 27'h00000005;  // Mass of Earth in kg (5.972e24)
parameter DT = 27'h0000000A; // Time step in seconds

// Declare signals for calculations
wire [26:0] r_squared, inv_radius_cubed, ax, ay, dx, dy;
reg [26:0] vx_fp, vy_fp, x_fp, y_fp; // Registers to store velocity and position in floating-point

// Floating-point operation modules instantiation
FpMul r_squared_calc(.iA(x_fp), .iB(x_fp), .oProd(r_squared));  // r^2 = x^2 + y^2 (part of calculation, y^2 added separately)
FpAdd add_y_squared(.iA(r_squared), .iB(y_fp * y_fp), .oSum(r_squared));  // Complete r^2 calculation
FpInvSqrt inv_sqrt_r(.iA(r_squared), .oInvSqrt(inv_radius_cubed)); // 1/r^3 calculation, inv sqrt outputs 1/r

// Acceleration calculations
FpMul calc_ax(.iA(x_fp), .iB(inv_radius_cubed * G * M), .oProd(ax)); // ax = -G*M*x/r^3
FpMul calc_ay(.iA(y_fp), .iB(inv_radius_cubed * G * M), .oProd(ay)); // ay = -G*M*y/r^3

// Update velocities and positions
always @(posedge clk or posedge rst) begin
    if (rst) begin
        // Initial conditions
        x_fp <= 27'h1A36E2; // Approximate 400,000 meters from Earth's surface in fp
        y_fp <= 0;
        vx_fp <= 0;
        vy_fp <= 27'h16A0A; // Initial tangential velocity for stable orbit in fp
    end else begin
        // Euler method to update velocity and position
        vx_fp <= vx_fp + ax * DT;
        vy_fp <= vy_fp + ay * DT;
        x_fp <= x_fp + vx_fp * DT;
        y_fp <= y_fp + vy_fp * DT;

        // Output the current position (scaled down for visualization)
        X <= x_fp >> 10;  // Scale down for display purposes
        Y <= y_fp >> 10;  // Scale down for display purposes
    end
end

endmodule
