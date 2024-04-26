module Orbital_Path (
    input clk,           // Clock input
    input rst,           // Reset input
    output reg [31:0] X, // X position output
    output reg [31:0] Y  // Y position output
);

// Constants and initial conditions
parameter G = 32'h0000005D;  // Gravitational constant in m^3 kg^-1 s^-2 (6.67430e-11)
parameter M = 32'h00000005;  // Mass of Earth in kg (5.972e24)
parameter DT = 32'h0000000A;  // Time step in seconds
parameter T = 3600;           // Total time in seconds (1 hour)

// Declare signals for calculations
wire [31:0] r_squared, r;
wire [31:0] ax, ay, inv_radius_squared, inv_radius, prod_x, prod_y;
wire [31:0] dvx, dvy, dx, dy;

// Convert floating-point to integer and vice versa
Int2Fp int2fp_x (.iInteger(x_int), .oA(x_fp));
Int2Fp int2fp_y (.iInteger(y_int), .oA(y_fp));
Int2Fp int2fp_vx (.iInteger(vx_int), .oA(vx_fp));
Int2Fp int2fp_vy (.iInteger(vy_int), .oA(vy_fp));
Fp2Int fp2int_x (.iA(x_fp), .oInteger(x_int));
Fp2Int fp2int_y (.iA(y_fp), .oInteger(y_int));
Fp2Int fp2int_vx (.iA(vx_fp), .oInteger(vx_int));
Fp2Int fp2int_vy (.iA(vy_fp), .oInteger(vy_int));

// Perform calculations
FpMul x_fp_squared_calc(.iA(x_fp), .iB(x_fp), .oProd(x_fp_squared));
FpMul y_fp_squared_calc(.iA(y_fp), .iB(y_fp), .oProd(y_fp_squared));
FpAdd r_squared_calc(.iCLK(clk), .iA(x_fp_squared), .iB(y_fp_squared), .oSum(r_squared));
FpInvSqrt inv_sqrt_r_squared_calc(.iCLK(clk), .iA(r_squared), .oInvSqrt(r));
FpMul inv_radius_cubed_calc(.iA(r), .iB(r), .oProd(inv_radius_squared));
FpMul inv_radius_calc(.iA(inv_radius_squared), .iB(r), .oProd(inv_radius));
FpMul prod_x_calc(.iA(x_fp), .iB(inv_radius_cubed), .oProd(prod_x));
FpMul prod_y_calc(.iA(y_fp), .iB(inv_radius_cubed), .oProd(prod_y));
FpMul ax_calc(.iA(prod_x), .iB(G * M), .oProd(ax));
FpMul ay_calc(.iA(prod_y), .iB(G * M), .oProd(ay));
FpMul ax_dt_calc(.iA(ax), .iB(DT), .oProd(dvx));
FpMul ay_dt_calc(.iA(ay), .iB(DT), .oProd(dvy));
FpAdd vx_fp_calc(.iCLK(clk), .iA(vx_fp), .iB(dvx), .oSum(vx_fp));
FpAdd vy_fp_calc(.iCLK(clk), .iA(vy_fp), .iB(dvy), .oSum(vy_fp));
FpMul vx_dt_calc(.iA(vx_fp), .iB(DT), .oProd(dx));
FpMul vy_dt_calc(.iA(vy_fp), .iB(DT), .oProd(dy));
FpAdd x_fp_calc(.iCLK(clk), .iA(x_fp), .iB(dx), .oSum(x_fp));
FpAdd y_fp_calc(.iCLK(clk), .iA(y_fp), .iB(dy), .oSum(y_fp));

// Other declarations...
reg [7:0] steps;  // Number of steps for simulation
reg [31:0] i; // Loop counter

// Array for storing X and Y positions
reg [31:0] X_array[0:steps-1]; // Array for storing X positions
reg [31:0] Y_array[0:steps-1]; // Array for storing Y positions

// Simulation time
always @(posedge clk) begin
    if (rst) begin
        // Set initial conditions
        x_fp <= $signed(32'h00026F90); // Initial x position in floating-point format
        y_fp <= 32'h00000000;          // Initial y position in floating-point format
        vx_fp <= 32'h00000000;         // Initial x velocity in floating-point format
        vy_fp <= $signed(sqrt(G * M / x_fp)); // Initial y velocity in floating-point format
        x_int <= 6.371e6 + 400000;     // Initial x position in integer format
        y_int <= 0;                     // Initial y position in integer format
        vx_int <= 0;                   // Initial x velocity in integer format
        vy_int <= sqrt(G * M / x_int); // Initial y velocity in integer format
        steps <= T / DT;               // Calculate number of steps for simulation
        i <= 0;                        // Initialize loop counter
    end else begin
        // Euler's method to update position and velocity
        if (i < steps) begin
            // Store positions for plotting
            X_array[i] <= x_int;
            Y_array[i] <= y_int;
            i <= i + 1;
        end
    end
end

// Plotting the orbit
always @(posedge clk) begin
    if (rst) begin
        X <= 0; // Reset X position output
        Y <= 0; // Reset Y position output
    end else begin
        if (i == steps) begin
            X <= X_array[steps-1]; // Output X position
            Y <= Y_array[steps-1]; // Output Y position
        end
    end
end

endmodule
