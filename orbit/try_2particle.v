module orbital_computation(
    input wire clk,
    input wire reset_n,
    output reg [31:0] px1,  // Screen coordinates of particle 1
    output reg [31:0] py1,
    output reg [31:0] px2,  // Screen coordinates of particle 2
    output reg [31:0] py2
);

    // Define fixed-point parameters
    parameter G = 32'd667430;  // Gravitational constant in fixed-point
    parameter M = 32'd597200000;  // Mass of Earth in fixed-point
    parameter dt = 32'd10;  // Time step in seconds (fixed-point)

    // Define particles' initial conditions (fixed-point)
    reg signed [31:0] x1, y1, vx1, vy1;
    reg signed [31:0] x2, y2, vx2, vy2;

    // Initialize the particles
    initial begin
        x1 = 32'd6771000;  // 6.371e6 + 400000 (Earth's radius + altitude of 400 km)
        y1 = 32'd0;
        vx1 = 32'd0;
        vy1 = 32'd7670;  // sqrt(G * M / x) approximated in fixed-point

        x2 = 32'd7171000;  // 6.371e6 + 800000
        y2 = 32'd0;
        vx2 = 32'd0;
        vy2 = 32'd7300;  // sqrt(G * M / x) approximated in fixed-point

        px1 = 32'd320;
        py1 = 32'd240;

        px2 = 32'd320;
        py2 = 32'd240;
    end

    // Update the orbital positions
    always @(posedge clk or negedge reset_n) begin
        if (!reset_n) begin
            // Reset the simulation
            x1 <= 32'd6771000;
            y1 <= 32'd0;
            vx1 <= 32'd0;
            vy1 <= 32'd7670;

            x2 <= 32'd7171000;
            y2 <= 32'd0;
            vx2 <= 32'd0;
            vy2 <= 32'd7300;

            px1 <= 32'd320;
            py1 <= 32'd240;

            px2 <= 32'd320;
            py2 <= 32'd240;
        end else begin
            // Update particle 1
            // Calculate acceleration components
            reg signed [63:0] r1_sqr = (x1 * x1) + (y1 * y1);
            reg signed [31:0] r1_cubed = r1_sqr * r1_sqr;
            reg signed [31:0] ax1 = (-G * M * x1) / r1_cubed;
            reg signed [31:0] ay1 = (-G * M * y1) / r1_cubed;

            // Update velocity
            vx1 <= vx1 + (ax1 * dt);
            vy1 <= vy1 + (ay1 * dt);

            // Update position
            x1 <= x1 + (vx1 * dt);
            y1 <= y1 + (vy1 * dt);

            // Convert to screen coordinates
            px1 <= (x1 >> 16) + 320;  // Assuming a scale factor
            py1 <= (y1 >> 16) + 240;

            // Update particle 2
            // Calculate acceleration components
            reg signed [63:0] r2_sqr = (x2 * x2) + (y2 * y2);
            reg signed [31:0] r2_cubed = r2_sqr * r2_sqr;
            reg signed [31:0] ax2 = (-G * M * x2) / r2_cubed;
            reg signed [31:0] ay2 = (-G * M * y2) / r2_cubed;

            // Update velocity
            vx2 <= vx2 + (ax2 * dt);
            vy2 <= vy2 + (ay2 * dt);

            // Update position
            x2 <= x2 + (vx2 * dt);
            y2 <= y2 + (vy2 * dt);

            // Convert to screen coordinates
            px2 <= (x2 >> 16) + 320;  // Assuming a scale factor
            py2 <= (y2 >> 16) + 240;
        end
    end
endmodule




module orbital_computation_wrapper (
    input wire clk,
    input wire reset_n,
    input wire [1:0] addr,  // 2-bit address for selecting which particle data to read
    output reg [31:0] data_out
);

    wire [31:0] px1, py1, px2, py2;

    orbital_computation u_orbital_computation (
        .clk(clk),
        .reset_n(reset_n),
        .px1(px1),
        .py1(py1),
        .px2(px2),
        .py2(py2)
    );

    always @(*) begin
        case (addr)
            2'b00: data_out = px1;
            2'b01: data_out = py1;
            2'b10: data_out = px2;
            2'b11: data_out = py2;
            default: data_out = 32'd0;
        endcase
    end
endmodule
