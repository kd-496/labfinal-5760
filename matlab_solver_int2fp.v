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

// Initial position and velocity (floating-point format)
reg signed [15:0] x_fp, y_fp, vx_fp, vy_fp;
// Initial position and velocity (integer format)
reg [31:0] x_int, y_int, vx_int, vy_int;
// Conversion from floating-point to integer
Int2Fp int2fp_x (.iInteger(x_int), .oA(x_fp));
Int2Fp int2fp_y (.iInteger(y_int), .oA(y_fp));
Int2Fp int2fp_vx (.iInteger(vx_int), .oA(vx_fp));
Int2Fp int2fp_vy (.iInteger(vy_int), .oA(vy_fp));
// Conversion from integer to floating-point
Fp2Int fp2int_x (.iA(x_fp), .oInteger(x_int));
Fp2Int fp2int_y (.iA(y_fp), .oInteger(y_int));
Fp2Int fp2int_vx (.iA(vx_fp), .oInteger(vx_int));
Fp2Int fp2int_vy (.iA(vy_fp), .oInteger(vy_int));

// Euler's method to update position and velocity
always @(posedge clk) begin
    if (rst) begin
        x_fp <= $signed(32'h00026F90); // Reset x position in floating-point format
        y_fp <= 32'h00000000;          // Reset y position in floating-point format
        vx_fp <= 32'h00000000;         // Reset x velocity in floating-point format
        vy_fp <= $signed(sqrt(G * M / x_fp)); // Circular orbit velocity in floating-point format
        x_int <= 6.371e6 + 400000;     // Reset x position in integer format
        y_int <= 0;                     // Reset y position in integer format
        vx_int <= 0;                   // Reset x velocity in integer format
        vy_int <= sqrt(G * M / x_int); // Circular orbit velocity in integer format
        // Conversion from integer to floating-point
        int2fp_x.iInteger <= x_int;
        int2fp_y.iInteger <= y_int;
        int2fp_vx.iInteger <= vx_int;
        int2fp_vy.iInteger <= vy_int;
    end else begin
        // Calculate radial distance
        wire [31:0] r, r_squared;
        FpMul x_fp_squared_calc(.iA(x_fp), .iB(x_fp), .oProd(r_squared));
        FpMul y_fp_squared_calc(.iA(y_fp), .iB(y_fp), .oProd(y_fp_squared));
        FpAdd r_squared_calc(.iCLK(clk), .iA(x_fp_squared), .iB(y_fp_squared), .oSum(r_squared));
        FpInvSqrt inv_sqrt_r_squared_calc(.iCLK(clk), .iA(r_squared), .oInvSqrt(r));
      
        // Calculate acceleration components
        wire [31:0] ax, ay;
        FpMul inv_radius_cubed_calc(.iA(r), .iB(r), .oProd(inv_radius_squared));
        FpMul inv_radius_calc(.iA(inv_radius_squared), .iB(r), .oProd(inv_radius));
        FpMul prod_x_calc(.iA(x_fp), .iB(inv_radius_cubed), .oProd(prod_x));
        FpMul prod_y_calc(.iA(y_fp), .iB(inv_radius_cubed), .oProd(prod_y));
        FpMul ax_calc(.iA(prod_x), .iB(G * M), .oProd(ax));
        FpMul ay_calc(.iA(prod_y), .iB(G * M), .oProd(ay));
      
        // Update velocities
        wire [31:0] dvx, dvy;
        FpMul ax_dt_calc(.iA(ax), .iB(DT), .oProd(dvx));
        FpMul ay_dt_calc(.iA(ay), .iB(DT), .oProd(dvy));
        FpAdd vx_fp_calc(.iCLK(clk), .iA(vx_fp), .iB(dvx), .oSum(vx_fp));
        FpAdd vy_fp_calc(.iCLK(clk), .iA(vy_fp), .iB(dvy), .oSum(vy_fp));
        
        // Update positions
        wire [31:0] dx, dy;
        FpMul vx_dt_calc(.iA(vx_fp), .iB(DT), .oProd(dx));
        FpMul vy_dt_calc(.iA(vy_fp), .iB(DT), .oProd(dy));
        FpAdd x_fp_calc(.iCLK(clk), .iA(x_fp), .iB(dx), .oSum(x_fp));
        FpAdd y_fp_calc(.iCLK(clk), .iA(y_fp), .iB(dy), .oSum(y_fp));
        
        // Conversion from floating-point to integer
        int2fp_x.iInteger <= x_int;
        int2fp_y.iInteger <= y_int;
        int2fp_vx.iInteger <= vx_int;
        int2fp_vy.iInteger <= vy_int;
    end
end

endmodule
