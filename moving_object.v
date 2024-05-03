module VGA_Controller(
    input clk,                    // Clock signal
    input reset,                  // Reset signal
    input [9:0] pixel_x,          // X-coordinate of the pixel
    input [8:0] pixel_y,          // Y-coordinate of the pixel
    input [23:0] pixel_color,     // 24-bit pixel color (RGB)
    output reg [7:0] VGA_R,       // VGA Red channel
    output reg [7:0] VGA_G,       // VGA Green channel
    output reg [7:0] VGA_B,       // VGA Blue channel
    output reg VGA_HS,            // Horizontal sync
    output reg VGA_VS,            // Vertical sync
    output VGA_BLANK_N,           // VGA blank signal
    output VGA_SYNC_N,            // VGA sync signal
    output VGA_CLK                // VGA clock signal
);

// VGA screen and timing constants
parameter H_RES = 640;           // Horizontal resolution
parameter V_RES = 480;           // Vertical resolution
parameter H_FP = 16;             // Horizontal front porch
parameter H_PW = 96;             // Horizontal pulse width
parameter H_BP = 48;             // Horizontal back porch
parameter V_FP = 10;             // Vertical front porch
parameter V_PW = 2;              // Vertical pulse width
parameter V_BP = 33;             // Vertical back porch
parameter H_TOTAL = H_RES + H_FP + H_PW + H_BP; // Total horizontal pixels
parameter V_TOTAL = V_RES + V_FP + V_PW + V_BP; // Total vertical lines

reg [9:0] h_counter = 0;         // Horizontal counter
reg [9:0] v_counter = 0;         // Vertical counter

assign VGA_BLANK_N = 1'b1;
assign VGA_SYNC_N = 1'b0;
assign VGA_CLK = clk;

always @(posedge clk or posedge reset) begin
    if (reset) begin
        h_counter <= 0;
        v_counter <= 0;
    end else begin
        // Horizontal counter logic
        if (h_counter < H_TOTAL - 1)
            h_counter <= h_counter + 1;
        else begin
            h_counter <= 0;
            // Vertical counter logic
            if (v_counter < V_TOTAL - 1)
                v_counter <= v_counter + 1;
            else
                v_counter <= 0;
        end
    end
end

always @(posedge clk) begin
    // Generate sync signals
    VGA_HS <= (h_counter < H_PW) ? 0 : 1;
    VGA_VS <= (v_counter < V_PW) ? 0 : 1;

    // Enable display within active region
    if (h_counter >= H_PW + H_BP && h_counter < H_PW + H_BP + H_RES &&
        v_counter >= V_PW + V_BP && v_counter < V_PW + V_BP + V_RES) begin
        // Set pixel color
        VGA_R <= (pixel_x == h_counter - (H_PW + H_BP) && pixel_y == v_counter - (V_PW + V_BP)) ? pixel_color[23:16] : 0;
        VGA_G <= (pixel_x == h_counter - (H_PW + H_BP) && pixel_y == v_counter - (V_PW + V_BP)) ? pixel_color[15:8] : 0;
        VGA_B <= (pixel_x == h_counter - (H_PW + H_BP) && pixel_y == v_counter - (V_PW + V_BP)) ? pixel_color[7:0] : 0;
    end else begin
        VGA_R <= 0;
        VGA_G <= 0;
        VGA_B <= 0;
    end
end

endmodule
