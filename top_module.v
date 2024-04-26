module Top_Module (
    input wire clock,
    input wire reset,
    output wire hsync,
    output wire vsync,
    output wire [7:0] red,
    output wire [7:0] green,
    output wire [7:0] blue,
    output wire sync,
    output wire clk_out,
    output wire blank
);

    // Instantiate Orbital_Path module
    Orbital_Path orbital_path_inst (
        .clk(clock),          // Connect clock signal
        .rst(reset),          // Connect reset signal
        .X(X),                // Connect X position output
        .Y(Y)                 // Connect Y position output
    );

    // Instantiate vga_driver module
    vga_driver vga_driver_inst (
        .clock(clock),        // Connect clock signal
        .reset(reset),        // Connect reset signal
        .color_in(color_in), // Connect color_in signal from Orbital_Path module
        .next_x(next_x),     // Connect next_x output from vga_driver module
        .next_y(next_y),     // Connect next_y output from vga_driver module
        .hsync(hsync),        // Connect hsync output from vga_driver module
        .vsync(vsync),        // Connect vsync output from vga_driver module
        .red(red),            // Connect red output from vga_driver module
        .green(green),        // Connect green output from vga_driver module
        .blue(blue),          // Connect blue output from vga_driver module
        .sync(sync),          // Connect sync output from vga_driver module
        .clk_out(clk_out),    // Connect clk_out output from vga_driver module
        .blank(blank)         // Connect blank output from vga_driver module
    );

    // Signal for color input to vga_driver module
    wire [7:0] color_in;
    // Set color_in to white (maximum intensity for each color component)
    assign color_in = (reset) ? 8'b00000000 : 8'b11111111; // White when not reset, black when reset

endmodule
