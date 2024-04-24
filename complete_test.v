module DE1_SoC_Computer (
    // Clock and Reset
    input CLOCK_50,
    input [3:0] KEY,

    // VGA Output
    output [7:0] VGA_R,
    output [7:0] VGA_G,
    output [7:0] VGA_B,
    output VGA_HS,
    output VGA_VS
);

    // Orbital Mechanics Parameters
    // Constants (using scaled fixed-point values)
    localparam integer G = 32'd67143; // Gravitational constant scaled down
    localparam integer M = 32'd5972; // Mass of Earth scaled down
    localparam integer dt = 10; // Time step in clock cycles

    // State variables for position and velocity (fixed-point)
    reg signed [31:0] x, y, vx, vy;
    reg signed [31:0] ax, ay; // Acceleration

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

// VGA controller module to handle VGA signal generation
module vga_controller(
    input clk,
    input reset_n,
    input [9:0] pixel_x, pixel_y,
    output reg [7:0] VGA_R, VGA_G, VGA_B,
    output reg VGA_HS, VGA_VS
);
    // VGA timing constants
    localparam H_DISPLAY = 640;
    localparam H_FRONT_PORCH = 16;
    localparam H_SYNC_PULSE = 96;
    localparam H_BACK_PORCH = 48;
    localparam H_TOTAL = H_DISPLAY + H_FRONT_PORCH + H_SYNC_PULSE + H_BACK_PORCH;

    localparam V_DISPLAY = 480;
    localparam V_FRONT_PORCH = 10;
    localparam V_SYNC_PULSE = 2;
    localparam V_BACK_PORCH = 33;
    localparam V_TOTAL = V_DISPLAY + V_FRONT_PORCH + V_SYNC_PULSE + V_BACK_PORCH;

    reg [10:0] h_count = 0, v_count = 0;

    always @(posedge clk) begin
        if (reset_n) begin
            // Increment horizontal counter
            if (h_count < (H_TOTAL - 1))
                h_count <= h_count + 1;
            else begin
                h_count <= 0;
                // Increment vertical counter
                if (v_count < (V_TOTAL - 1))
                    v_count <= v_count + 1;
                else
                    v_count <= 0;
            end

            // Generate sync signals
            VGA_HS <= (h_count >= (H_DISPLAY + H_FRONT_PORCH) && h_count < (H_DISPLAY + H_FRONT_PORCH + H_SYNC_PULSE)) ? 0 : 1;
            VGA_VS <= (v_count >= (V_DISPLAY + V_FRONT_PORCH) && v_count < (V_DISPLAY + V_FRONT_PORCH + V_SYNC_PULSE)) ? 0 : 1;
        end
        else begin
            h_count <= 0;
            v_count <= 0;
        end

        // Generate video signal
        if ((h_count < H_DISPLAY) && (v_count < V_DISPLAY)) begin
            if (pixel_x == h_count && pixel_y == v_count) begin
                VGA_R <= 255;
                VGA_G <= 255;
                VGA_B <= 255;
            end else begin
                VGA_R <= 0;
                VGA_G <= 0;
                VGA_B <= 0;
            end
        end else begin
            VGA_R <= 0;
            VGA_G <= 0;
            VGA_B <= 0;
        end
    end
endmodule
