module DE1_SoC_Computer (
    input CLOCK_50,
    input [3:0] KEY,
    output [7:0] VGA_R,
    output [7:0] VGA_G,
    output [7:0] VGA_B,
    output VGA_HS,
    output VGA_VS
);

    // Fixed-point constants
    localparam integer G = 32'd67143;  // Gravitational constant scaled down
    localparam integer M = 32'd5972;   // Mass of Earth scaled down
    localparam integer dt = 10;        // Time step in clock cycles

    // Registers for position and velocity (x, y, vx, vy are signed 32-bit)
    reg signed [31:0] x, y, vx, vy;
    reg signed [31:0] ax, ay; // Acceleration

    // Computation variables (large enough to handle multiplications)
    reg signed [63:0] r_squared;
    reg signed [31:0] r_cubed;

    // Initial conditions
    initial begin
        x = 32'd6371000 + 32'd400000; // Initial x: radius of Earth + altitude
        y = 32'd0;  // Initial y
        vx = 32'd0; // Initial vx
        vy = 32'd7758; // Initial vy (orbital velocity at 400 km altitude)
    end

    // Update positions and velocities
    always @(posedge CLOCK_50) begin
        if (!KEY[0]) begin // Reset to initial conditions on KEY[0]
            x <= 32'd6371000 + 32'd400000;
            y <= 32'd0;
            vx <= 32'd0;
            vy <= 32'd7758;
        end else begin
            // Calculate radial distance squared and cubed
            r_squared <= x * x + y * y;
            r_cubed <= r_squared * $signed($sqrt(r_squared));

            // Compute acceleration
            ax <= -G * M * x / r_cubed;
            ay <= -G * M * y / r_cubed;

            // Update velocities
            vx <= vx + ax * dt;
            vy <= vy + ay * dt;

            // Update positions
            x <= x + vx * dt;
            y <= y + vy * dt;
        end
    end

    // VGA signal generation
    wire [9:0] pixel_x = (x >> 16) % 640;
    wire [9:0] pixel_y = (y >> 16) % 480;

    vga_controller vga_ctrl (
        .clk(CLOCK_50),
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
