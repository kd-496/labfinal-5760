module vga_driver (
    input wire clock,         // 25 MHz
    input wire reset,         // Active high
    input [7:0] color_in,     // Pixel color data (RRRGGGBB)
    output [9:0] next_x,      // x-coordinate of NEXT pixel that will be drawn
    output [9:0] next_y,      // y-coordinate of NEXT pixel that will be drawn
    output reg hsync,         // HSYNC (to VGA connector)
    output reg vsync,         // VSYNC (to VGA connector)
    output [7:0] red,         // RED (to resistor DAC VGA connector)
    output [7:0] green,       // GREEN (to resistor DAC to VGA connector)
    output [7:0] blue,        // BLUE (to resistor DAC to VGA connector)
    output sync,              // SYNC to VGA connector
    output clk,               // CLK to VGA connector
    output blank              // BLANK to VGA connector
);

    // Horizontal and Vertical constants for 640x480 @ 60Hz
    parameter H_ACTIVE = 640;
    parameter H_FRONT_PORCH = 16;
    parameter H_SYNC_PULSE = 96;
    parameter H_BACK_PORCH = 48;
    parameter H_TOTAL = H_ACTIVE + H_FRONT_PORCH + H_SYNC_PULSE + H_BACK_PORCH;

    parameter V_ACTIVE = 480;
    parameter V_FRONT_PORCH = 10;
    parameter V_SYNC_PULSE = 2;
    parameter V_BACK_PORCH = 33;
    parameter V_TOTAL = V_ACTIVE + V_FRONT_PORCH + V_SYNC_PULSE + V_BACK_PORCH;

    reg [9:0] h_count = 0;
    reg [9:0] v_count = 0;

    always @(posedge clock) begin
        if (reset) begin
            h_count <= 0;
            v_count <= 0;
        end else begin
            if (h_count < H_TOTAL - 1) begin
                h_count <= h_count + 1;
            end else begin
                h_count <= 0;
                if (v_count < V_TOTAL - 1) begin
                    v_count <= v_count + 1;
                end else begin
                    v_count <= 0;
                end
            end
        end

        // Generate HSYNC and VSYNC signals
        hsync <= (h_count >= H_ACTIVE + H_FRONT_PORCH) && (h_count < H_ACTIVE + H_FRONT_PORCH + H_SYNC_PULSE);
        vsync <= (v_count >= V_ACTIVE + V_FRONT_PORCH) && (v_count < V_ACTIVE + V_FRONT_PORCH + V_SYNC_PULSE);
    end

    // Output visible signal during active video time
    assign red = (h_count < H_ACTIVE && v_count < V_ACTIVE) ? color_in[7:5] << 5 : 0;
    assign green = (h_count < H_ACTIVE && v_count < V_ACTIVE) ? color_in[4:2] << 5 : 0;
    assign blue = (h_count < H_ACTIVE && v_count < V_ACTIVE) ? color_in[1:0] << 6 : 0;

    // Clock and Sync output
    assign clk = clock;
    assign sync = 0; // Typically not used in modern VGA, but depends on the monitor
    assign blank = ~(hsync | vsync);

    // Coordinate outputs for external usage
    assign next_x = h_count;
    assign next_y = v_count;
endmodule
